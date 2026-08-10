/*
 * Original code from Littleton Robotics (Team 6328) - 2026 Season
 * Modified by Team [10114]
 * * Licensed under the MIT License.
 */

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Degree;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.FieldConstants.siteConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.ShotSolver;
import frc.robot.util.FIeldHelper.AllianceFlipUtil;
import frc.robot.util.RobotStatus.RobotStatus;

public class ShooterCalculator {
        private final RobotStatus robotStatus;
        private final CommandSwerveDrivetrain drive;
        
        // 查表法物件保留作為靜態備案與相容性使用
        private final InterpolatingTreeMap<Double, Angle> hoodMap;
        private final InterpolatingTreeMap<Double, AngularVelocity> rollMap;
        private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
        private final InterpolatingTreeMap<Double, AngularVelocity> ToAillancerollMap;
        private static final InterpolatingDoubleTreeMap ToAillancetimeOfFlightMap = new InterpolatingDoubleTreeMap();
        
        private final double time_error = 0.0;
        private final double phaseDelay = 0.03 + time_error;
        public static Transform3d robotToTurret = new Transform3d(0.2, 0.0, 0.44, Rotation3d.kZero);

        private static final Angle Hood_MAX_RADS = ShooterConstants.Hood_MAX_LIMIT;

        public ShooterCalculator(CommandSwerveDrivetrain drive, RobotStatus robotStatus) {
                this.drive = drive;
                this.robotStatus = robotStatus;

                hoodMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                (start, end, t) -> {
                                        double startVal = start.in(Degree);
                                        double endVal = end.in(Degree);
                                        return Degree.of(MathUtil.interpolate(startVal, endVal, t));
                                });
                rollMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                (start, end, t) -> {
                                        double startVal = start.in(RotationsPerSecond);
                                        double endVal = end.in(RotationsPerSecond);
                                        return RotationsPerSecond.of(MathUtil.interpolate(startVal, endVal, t));
                                });
                ToAillancerollMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                (start, end, t) -> {
                                        double startVal = start.in(RotationsPerSecond);
                                        double endVal = end.in(RotationsPerSecond);
                                        return RotationsPerSecond.of(MathUtil.interpolate(startVal, endVal, t));
                                });

                // 在這裡可以保留你的 fallback 查表數據...
                // (省略部分 map.put 以保持版面簡潔，你的舊數據已經在之前的版本中)
        }

        // ✨ 完整 9 個參數的 ShootingState，完美支援走射前饋與砲塔控制
        public record ShootingState(
                        Rotation2d turretRelativeAngle, // 🟢 砲塔相對於車頭的目標角度
                        Angle HoopAngle,                // 🟢 機構仰角
                        AngularVelocity FlywheelRPS,    // 🟢 飛輪轉速
                        double chassisOmegaRadsPerSec,  // FF: 底盤角速度
                        double fieldVelocityX,          // FF: 底盤 X 速度
                        double fieldVelocityY,          // FF: 底盤 Y 速度
                        double deltaX,                  // FF: 目標 X 距離差
                        double deltaY,                  // FF: 目標 Y 距離差
                        double timeOfFlightSec          // 🟢 預測飛行時間 (ToF)
        ) {}

        // ========================================================================================
        // 🚀 主目標 (Hub) 射擊計算 - 使用 ShotSolver
        // ========================================================================================
        public ShootingState calculateShootingToHub() {
                Pose2d estimatedPose = drive.getPose2d();
                ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

                // 延遲補償
                estimatedPose = estimatedPose.exp(new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

                ChassisSpeeds fieldVelocitySpeeds = drive.getFieldVelocity();
                Translation2d fieldVelocity = new Translation2d(
                                fieldVelocitySpeeds.vxMetersPerSecond,
                                fieldVelocitySpeeds.vyMetersPerSecond);

                // 速度向量紅藍方修正
                if (AllianceFlipUtil.shouldFlip()) {
                        fieldVelocity = new Translation2d(-fieldVelocity.getX(), -fieldVelocity.getY());
                }

                // 目標位置與 3D 高度
                Translation2d target2d = AllianceFlipUtil.apply(siteConstants.topCenterPoint.toTranslation2d());
                Translation3d target3d = new Translation3d(target2d.getX(), target2d.getY(), 1.83);
                Pose3d robotPose3d = new Pose3d(estimatedPose);

                // 呼叫 ShotSolver
                ShotSolver.Solution solution = ShotSolver.solve(
                                robotPose3d,
                                fieldVelocity,
                                fieldVelocitySpeeds.omegaRadiansPerSecond,
                                target3d,
                                ShotSolver.Calibration.identity());

                if (solution.valid()) {
                        // 處理場地絕對角度與紅藍翻轉
                        Rotation2d targetFieldAngle = estimatedPose.getRotation()
                                        .plus(Rotation2d.fromRadians(solution.turretYawRad()));
                        if (AllianceFlipUtil.shouldFlip()) {
                                targetFieldAngle = Rotation2d.fromDegrees(targetFieldAngle.getDegrees() - 180.0);
                        }

                        // 將場地絕對角度轉為「相對車頭角度」
                        Rotation2d targetRelativeAngle = targetFieldAngle.minus(estimatedPose.getRotation());

                        // 前饋使用的距離差
                        Translation2d vectorToTarget = target2d.minus(estimatedPose.getTranslation());

                        // 🚨 仰角轉換：將物理引擎的「90度水平」轉換為硬體的「0度水平」
                        double hoodDegrees = 90.0 - Math.toDegrees(solution.hoodRad());

                        Logger.recordOutput("ShotSolver_Hub/ToF", solution.tofSec());
                        Logger.recordOutput("ShotSolver_Hub/ExitSpeedMps", solution.exitSpeedMps());

                        return new ShootingState(
                                        targetRelativeAngle,                           
                                        Degree.of(hoodDegrees),                        
                                        RotationsPerSecond.of(solution.flywheelRps()), 
                                        fieldVelocitySpeeds.omegaRadiansPerSecond,     
                                        fieldVelocitySpeeds.vxMetersPerSecond,         
                                        fieldVelocitySpeeds.vyMetersPerSecond,         
                                        vectorToTarget.getX(),                         
                                        vectorToTarget.getY(),                         
                                        solution.tofSec()                              
                        );
                } else {
                        System.err.println("ShotSolver 判定 Hub 射擊無法達成!");
                        return getFallbackState(estimatedPose);
                }
        }

        // ========================================================================================
        // 🚀 傳球 (Alliance) 射擊計算 - 使用 ShotSolver
        // ========================================================================================
        public ShootingState calculateShootingToAlliance() {
                Pose2d estimatedPose = drive.getPose2d();
                ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

                // 延遲補償
                estimatedPose = estimatedPose.exp(new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

                ChassisSpeeds fieldVelocitySpeeds = drive.getFieldVelocity();
                Translation2d fieldVelocity = new Translation2d(
                                fieldVelocitySpeeds.vxMetersPerSecond,
                                fieldVelocitySpeeds.vyMetersPerSecond);

                // 速度向量紅藍方修正
                if (AllianceFlipUtil.shouldFlip()) {
                        fieldVelocity = new Translation2d(-fieldVelocity.getX(), -fieldVelocity.getY());
                }

                // 傳球目標位置
                Translation2d target2d;
                if (robotStatus.getVerticalSide() == RobotStatus.VerticalSide.TOP) {
                        target2d = AllianceFlipUtil.apply(siteConstants.topLeftCenterPoint.toTranslation2d());
                } else {
                        target2d = AllianceFlipUtil.apply(siteConstants.topRightCenterPoint.toTranslation2d());
                }

                Translation3d target3d = new Translation3d(target2d.getX(), target2d.getY(), 1.83);
                Pose3d robotPose3d = new Pose3d(estimatedPose);

                // 呼叫 ShotSolver
                ShotSolver.Solution solution = ShotSolver.solve(
                                robotPose3d,
                                fieldVelocity,
                                fieldVelocitySpeeds.omegaRadiansPerSecond,
                                target3d,
                                ShotSolver.Calibration.identity());

                if (solution.valid()) {
                        Rotation2d targetFieldAngle = estimatedPose.getRotation()
                                        .plus(Rotation2d.fromRadians(solution.turretYawRad()));
                        if (AllianceFlipUtil.shouldFlip()) {
                                targetFieldAngle = Rotation2d.fromDegrees(targetFieldAngle.getDegrees() - 180.0);
                        }
                        
                        Rotation2d targetRelativeAngle = targetFieldAngle.minus(estimatedPose.getRotation());
                        Translation2d vectorToTarget = target2d.minus(estimatedPose.getTranslation());

                        // 🚨 仰角轉換：將物理引擎的「90度水平」轉換為硬體的「0度水平」
                        double hoodDegrees = 90.0 - Math.toDegrees(solution.hoodRad());

                        Logger.recordOutput("ShotSolver_Alliance/ToF", solution.tofSec());
                        Logger.recordOutput("ShotSolver_Alliance/ExitSpeedMps", solution.exitSpeedMps());

                        return new ShootingState(
                                        targetRelativeAngle,                           
                                        Degree.of(hoodDegrees),                        
                                        RotationsPerSecond.of(solution.flywheelRps()), 
                                        fieldVelocitySpeeds.omegaRadiansPerSecond,     
                                        fieldVelocitySpeeds.vxMetersPerSecond,         
                                        fieldVelocitySpeeds.vyMetersPerSecond,         
                                        vectorToTarget.getX(),                         
                                        vectorToTarget.getY(),                         
                                        solution.tofSec()                              
                        );
                } else {
                        System.err.println("ShotSolver 判定傳球無法達成 (超出物理極限)!");
                        return getFallbackState(estimatedPose);
                }
        }

        // ========================================================================================
        // 🛡️ 防呆備用狀態 (Fallback)
        // ========================================================================================
        private ShootingState getFallbackState(Pose2d currentPose) {
                return new ShootingState(
                                Rotation2d.fromDegrees(0), // 預設對正車頭
                                Hood_MAX_RADS,             // 把仰角抬到最高確保安全
                                RotationsPerSecond.of(0),  // 不發射
                                0.0, 
                                0.0, 
                                0.0, 
                                0.0, 
                                0.0, 
                                0.0                        // ToF 預設 0
                );
        }
}