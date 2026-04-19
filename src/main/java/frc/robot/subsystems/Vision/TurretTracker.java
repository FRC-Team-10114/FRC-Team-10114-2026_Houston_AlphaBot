package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.Turret.TurretIO;


public class TurretTracker extends SubsystemBase {
    private final TurretKalmanFilter2D turretKF = new TurretKalmanFilter2D();
    private final CommandSwerveDrivetrain drivetrain;
    private final TurretIO turret;
    public Transform3d fieldToCameraTrans;

    // 🌟 專屬於砲台的視覺硬體
    private final PhotonCamera turretCamera;
    private final Transform3d turretToCamera;

    // 雙重時光機 1：砲台相對角度歷史
    private final TimeInterpolatableBuffer<Rotation2d> turretEncoderHistory = TimeInterpolatableBuffer
            .createBuffer(1.0);

    // 物理安裝參數：砲台旋轉中心 相對於 機器人中心 的位移
    private final Translation2d robotToTurretPos = new Translation2d(0.15, 0.0);

    /**
     * 🌟 終極建構子：把相機直接綁定在這個追蹤器上
     * 
     * @param cameraName     PhotonVision 網頁上設定的砲台相機名稱 (例如 "TurretCam")
     * @param turretToCamera 相機相對於砲台中心的安裝偏移量
     */
    public TurretTracker(
            CommandSwerveDrivetrain drivetrain,
            TurretIO turretIO,
            String cameraName,
            Transform3d turretToCamera) {
        this.drivetrain = drivetrain;
        this.turret = turretIO;

        // 初始化硬體
        this.turretCamera = new PhotonCamera(cameraName);
        this.turretToCamera = turretToCamera;
    }

    @Override
    public void periodic() {
        // 1. 物理慣性推演與記錄編碼器歷史
        turretKF.predict();
        turretEncoderHistory.addSample(Timer.getTimestamp(), getRawEncoderRotation());

        // 2. 🌟 砲台子系統自己讀取鏡頭！完全不依賴外部的 Vision 迴圈！
        processCameraData();
    }

    /**
     * 🌟 內建的視覺處理引擎
     */
private void processCameraData() {
        // 1. 取得最新影像
        var result = turretCamera.getLatestResult();

        // =========================================================
        // 👉 就是這行！你剛剛漏了這行，所以下面才會不認得 multiTagResultOpt
        // =========================================================
        Optional<MultiTargetPNPResult> multiTagResultOpt = result.getMultiTagResult();

        // 2. 嚴格安全檢查：必須有目標、數量>=2，且盒子裡面真的有算出資料 (.isPresent)
        if (result.hasTargets() && result.getTargets().size() >= 2 && multiTagResultOpt.isPresent()) {
            
            double timestamp = result.getTimestampSeconds();
            int numTags = result.getTargets().size();
            
            // 計算標籤平均距離 (用來做信任度衰減)
            double avgDist = 0;
            for(var target : result.getTargets()) {
                avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
            }
            avgDist /= numTags;

            // 3. 完美解開盒子並取出 Transform3d
            MultiTargetPNPResult pnpResult = multiTagResultOpt.get();
            
            // 取得相機絕對座標 (如果這行報錯，請改成 pnpResult.best)
            Transform3d fieldToCameraTrans = pnpResult.estimatedPose.best;
            
            // 4. 將動態矩陣 (Transform3d) 轉換成絕對座標 (Pose3d)
            Pose3d fieldToCamera = new Pose3d(
                fieldToCameraTrans.getTranslation(),
                fieldToCameraTrans.getRotation()
            );

            // 5. 進入核心時光機對齊與濾波邏輯
            alignClocksAndUpdate(fieldToCamera, timestamp, avgDist, numTags);
        }
    }

    /**
     * 核心邏輯：同步雙時光機，計算誤差，並回饋底盤
     */
    private void alignClocksAndUpdate(Pose3d fieldToCamera, double timestamp, double avgDist, int numTags) {
        var robotPoseAtTimeOpt = drivetrain.getPoseAtTimestamp(timestamp);
        var turretEncoderAtTimeOpt = turretEncoderHistory.getSample(timestamp);

        // 如果找不到歷史數據，為了安全直接捨棄這幀影像
        if (robotPoseAtTimeOpt.isEmpty() || turretEncoderAtTimeOpt.isEmpty())
            return;

        Pose2d robotPoseAtTime = robotPoseAtTimeOpt.get();
        Rotation2d turretEncoderAtTime = turretEncoderAtTimeOpt.get();

        // 消除相機視差，算出那一刻的「砲台絕對角度」
        Pose3d fieldToTurretAtTime = fieldToCamera.transformBy(turretToCamera.inverse());
        Rotation2d turretAbsYawAtTime = fieldToTurretAtTime.getRotation().toRotation2d();

        // 計算純機械誤差 (Offset) 並修正 KF
        Rotation2d visionRelativeAngle = turretAbsYawAtTime.minus(robotPoseAtTime.getRotation());
        Rotation2d offsetError = visionRelativeAngle.minus(turretEncoderAtTime);

        // 動態標準差：距離越遠，KF 越不相信視覺
        double turretStdDevDegrees = 1.0 + Math.pow(avgDist, 1.2);
        turretKF.updateWithVision(offsetError.getRadians(), Units.degreesToRadians(turretStdDevDegrees));

        // --- 反向校正底盤 (Vision-to-Swerve Correction) ---
        Rotation2d filteredTurretRelativeRot = Rotation2d.fromRadians(turretKF.getEstimatedAngleRad());
        Pose2d correctedRobotPose = calculateCorrectedRobotPose(fieldToCamera, filteredTurretRelativeRot);

        // 算出底盤專用的信任度並餵給底盤
        double xyStdDevCoef = 0.01;
        double weightFactor = Math.pow(avgDist, 1.2) / Math.pow(Math.max(1, numTags), 2.0);
        double xyStdDev = xyStdDevCoef * weightFactor;
        double thetaStdDev = (numTags >= 2) ? (0.01 * weightFactor) : 9999999.0;

        Vector<N3> drivetrainStdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
        drivetrain.addVisionMeasurement(correctedRobotPose, timestamp, drivetrainStdDevs);
    }

    /**
     * 逆向運動學：反推機器人中心
     */
    private Pose2d calculateCorrectedRobotPose(Pose3d fieldToCamera, Rotation2d filteredTurretRelativeRot) {
        Pose3d fieldToTurret = fieldToCamera.transformBy(turretToCamera.inverse());
        Translation2d turretFieldTranslation = fieldToTurret.getTranslation().toTranslation2d();
        Rotation2d turretAbsoluteRot = fieldToTurret.getRotation().toRotation2d();

        Rotation2d robotAbsoluteRot = turretAbsoluteRot.minus(filteredTurretRelativeRot);
        Translation2d robotToTurretFieldOffset = robotToTurretPos.rotateBy(robotAbsoluteRot);
        Translation2d robotFieldTranslation = turretFieldTranslation.minus(robotToTurretFieldOffset);

        return new Pose2d(robotFieldTranslation, robotAbsoluteRot);
    }

    /**
     * 內部方法：原始編碼器角度
     */
    private Rotation2d getRawEncoderRotation() {
        return new Rotation2d(this.turret.getAngle());
    }

    /**
     * 取得「濾波矯正後」的砲台相對角度
     */
    public Rotation2d getEstimatedRelativeRotation() {
        return getRawEncoderRotation().plus(Rotation2d.fromRadians(turretKF.getEstimatedAngleRad()));
    }

    // ==========================================
    // 終極瞄準與黑盒子 API 區
    // ==========================================

    /**
     * 🌟 終極瞄準計算：算出馬達「真正該去」的編碼器角度 (供外部指令呼叫)
     */
    public Rotation2d calculateMotorTarget(Translation2d targetFieldPos) {
        Pose2d robotPose = drivetrain.getPose2d();
        Rotation2d absoluteAimAngle = new Rotation2d(Math.atan2(
                targetFieldPos.getY() - robotPose.getY(),
                targetFieldPos.getX() - robotPose.getX()));

        Rotation2d desiredRelativeAngle = absoluteAimAngle.minus(robotPose.getRotation());
        Rotation2d offsetError = getEstimatedRelativeRotation().minus(getRawEncoderRotation());
        Rotation2d targetMotorAngle = desiredRelativeAngle.minus(offsetError);

        double currentMotorDeg = getRawEncoderRotation().getDegrees();
        double optimizedTargetDeg = MathUtil.inputModulus(
                targetMotorAngle.getDegrees(),
                currentMotorDeg - 180.0,
                currentMotorDeg + 180.0);

        return Rotation2d.fromDegrees(optimizedTargetDeg);
    }

    public Pose2d getTurretFieldPose() {
        Pose2d robotPose = drivetrain.getPose2d();
        Translation2d turretFieldTranslation = robotPose.getTranslation()
                .plus(robotToTurretPos.rotateBy(robotPose.getRotation()));
        return new Pose2d(turretFieldTranslation, Rotation2d.fromDegrees(getTurretAbsoluteAngle360()));
    }

    public double getTurretAbsoluteAngle360() {
        Rotation2d robotRotation = drivetrain.getRotation();
        Rotation2d turretRelativeRotation = getEstimatedRelativeRotation();
        Rotation2d absoluteRotation = robotRotation.plus(turretRelativeRotation);
        return MathUtil.inputModulus(absoluteRotation.getDegrees(), 0.0, 360.0);
    }
}