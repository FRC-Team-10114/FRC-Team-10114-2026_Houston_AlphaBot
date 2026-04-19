package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.google.gson.annotations.Until;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class PhotonVision extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private boolean NeedResetPose = false;
    private int m_lastTagId = -1;

    private static class CamWrapper {
        final String name;
        final PhotonCamera cam;
        final PhotonPoseEstimator estimator;

        CamWrapper(String name, PhotonCamera cam, PhotonPoseEstimator estimator) {
            this.name = name;
            this.cam = cam;
            this.estimator = estimator;
        }
    }

    private final double borderPixels = PhotonVisionConstants.borderPixels;
    private final double maxSingleTagDistanceMeters = PhotonVisionConstants.maxSingleTagDistanceMeters;

    private final List<CamWrapper> cams = new ArrayList<>();

    public PhotonVision(CommandSwerveDrivetrain drive, Map<String, Transform3d> cameraTransforms) {
        this.drivetrain = drive;

        if (Robot.isSimulation()) {
            System.out.println("Simulation mode detected - Skipping PhotonVision initialization");
            return;
        }

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        cameraTransforms.forEach((name, transform) -> {
            PhotonCamera cam = new PhotonCamera(name);
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                    fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    transform);
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            cams.add(new CamWrapper(name, cam, estimator));
        });
    }

    @Override
    public void periodic() {
        updateVision();
    }

    public void NeedResetPoseEvent() {
        this.NeedResetPose = true;
    }

private void updateVision() {
        // 取得機器人當前位姿 (里程計)，用於協助消歧義
        Pose2d currentRobotPose = drivetrain.getPose2d();

        for (CamWrapper cw : cams) {
            for (PhotonPipelineResult result : cw.cam.getAllUnreadResults()) {

                // 基礎資訊更新
                if (result.hasTargets()) {
                    m_lastTagId = result.getBestTarget().getFiducialId();
                }

                // 🌟 升級 1：設定參考位姿 (Reference Pose)
                // 這會強迫 PhotonPoseEstimator 在遇到單標籤兩個解時，選擇離 currentRobotPose 較近的那個
                cw.estimator.setReferencePose(currentRobotPose);

                Optional<EstimatedRobotPose> poseOpt = cw.estimator.update(result);
                if (poseOpt.isEmpty())
                    continue;

                EstimatedRobotPose est = poseOpt.get();
                Pose3d cameraRobotPose3d = est.estimatedPose;
                double resultTimeSec = est.timestampSeconds;

                // ---------------------------------------------------------
                // 過濾 A：旋轉速度過快時丟棄 (避免捲動快門/動態模糊)
                // ---------------------------------------------------------
                double rotSpeed = Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
                if (rotSpeed > PhotonVisionConstants.maxYawRate)
                    continue;

                // ---------------------------------------------------------
                // 基礎運算：標籤數量與平均距離
                // ---------------------------------------------------------
                int numTags = est.targetsUsed.size();
                double avgDist = 0.0;
                for (var tgt : est.targetsUsed) {
                    avgDist += tgt.getBestCameraToTarget().getTranslation().getNorm();
                }
                if (numTags > 0) {
                    avgDist /= numTags;
                }

                // ---------------------------------------------------------
                // 過濾 B：單 Tag 有效性檢查
                // ---------------------------------------------------------
                if (numTags == 1) { // 修改為 == 1 較為精確
                    // 如果只有一個 Tag，距離太遠則丟棄
                    if (avgDist > maxSingleTagDistanceMeters)
                        continue;

                    // 由於上面加入了 setReferencePose，我們可以稍微放寬 Ambiguity 的限制
                    // 讓系統在稍微模糊但能被參考位姿救回的情況下依然運作。
                    // 如果你發現還是會跳動，可以改回 0.2
                    if (result.getBestTarget() != null && result.getBestTarget().getPoseAmbiguity() > 0.3)
                        continue;
                }

                // ---------------------------------------------------------
                // 🌟 升級 2：計算動態標準差 (Team 6328 經驗公式)
                // 公式: StdDev = coefficient * (distance^1.2) / (numTags^2)
                // ---------------------------------------------------------
                Vector<N3> stdDevs;
                
                if (this.NeedResetPose) {
                    // 硬重置時，極度信任視覺
                    stdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1));
                    this.NeedResetPose = false;
                } else {
                    // 基礎常數 (你可以根據實測微調這些數字)
                    double xyStdDevCoef = 0.01; 
                    double thetaStdDevCoef = 0.01;
                    
                    // 計算基於距離和數量的衰減權重
                    // 注意：Math.max(1, numTags) 是為了避免除以 0，雖然前面已經過濾過了
                    double weightFactor = Math.pow(avgDist, 1.2) / Math.pow(Math.max(1, numTags), 2.0);
                    
                    double xyStdDev = xyStdDevCoef * weightFactor;

                    if (numTags >= 2) {
                        // 多標籤：信任視覺的角度
                        double thetaStdDev = thetaStdDevCoef * weightFactor;
                        stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
                    } else {
                        // 單標籤：極度不信任視覺角度，強迫依賴陀螺儀
                        stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0);
                    }
                }

                Logger.recordOutput("Vision/cameraRobotPose_" + cw.cam.getName(), cameraRobotPose3d);
                
                // 加入到里程計融合
                drivetrain.addVisionMeasurement(
                        cameraRobotPose3d.toPose2d(),
                        resultTimeSec,
                        stdDevs);
            }
        }
    }

    /**
     * 用於自動階段初始化或特殊情況，尋找最可信的 Vision Pose 並強制覆蓋 Odometry
     */
    public boolean resetPoseToVision() {
        Pose2d bestPose = null;
        double minScore = 99999.0; // 分數越低越好

        for (CamWrapper cw : cams) {
            PhotonPipelineResult result = cw.cam.getLatestResult();
            if (!result.hasTargets())
                continue;

            Optional<EstimatedRobotPose> poseOpt = cw.estimator.update(result);
            if (poseOpt.isEmpty())
                continue;

            EstimatedRobotPose est = poseOpt.get();
            Pose3d pose3d = est.estimatedPose;

            // 1. 高度檢查
            if (Math.abs(pose3d.getZ()) > 0.5)
                continue;

            // 2. 計算 Tag 資訊
            int numTags = est.targetsUsed.size();
            double avgDist = 0.0;
            for (var tgt : est.targetsUsed) {
                avgDist += tgt.getBestCameraToTarget().getTranslation().getNorm();
            }
            if (numTags > 0)
                avgDist /= numTags;

            // 3. 過濾模糊單 Tag
            if (numTags == 1) {
                var bestTarget = result.getBestTarget();
                if (bestTarget != null && bestTarget.getPoseAmbiguity() > 0.2)
                    continue;
            }

            // 4. 評分 (多 Tag 優先，近距離優先)
            double currentScore;
            if (numTags >= 2) {
                currentScore = avgDist; // 多 Tag 直接比距離
            } else {
                currentScore = 100.0 + avgDist; // 單 Tag 分數加權，讓它永遠輸給多 Tag
            }

            // 更新最佳結果
            if (currentScore < minScore) {
                minScore = currentScore;
                bestPose = pose3d.toPose2d();
            }
        }

        if (bestPose != null) {
            drivetrain.resetPose(bestPose);
            return true;
        }

        return false;
    }

    public int getAprilTagId() {
        return m_lastTagId;
    }

    private boolean filterByZ(Pose3d pose3d) {
        return Math.abs(pose3d.getZ()) < PhotonVisionConstants.maxZ;
    }

    public boolean isCameraConnected(String cameraName) {
        for (CamWrapper cw : cams) {
            if (cw.name.equals(cameraName)) {
                return cw.cam.isConnected();
            }
        }
        return false;
    }

    /**
     * 檢查是否至少有一台相機連線正常
     */
    public boolean isAnyCameraConnected() {
        for (CamWrapper cw : cams) {
            if (cw.cam.isConnected())
                return true;
        }
        return false;
    }
}