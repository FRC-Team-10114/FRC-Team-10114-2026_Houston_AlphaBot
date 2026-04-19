/*
 * Original code from Littleton Robotics (Team 6328) - 2026 Season
 * Modified by Team [10114]
 * * Licensed under the MIT License.
 */

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.FieldConstants.siteConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.FIeldHelper.AllianceFlipUtil;
import frc.robot.util.RobotStatus.RobotStatus;

public class ShooterCalculator {
        private final RobotStatus robotStatus;
        private final CommandSwerveDrivetrain drive;
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
                                        // 1. 把單位轉成 double (用 Radians 或 Degrees 都可以，統一就好)
                                        double startVal = start.in(Degree);
                                        double endVal = end.in(Degree);
                                        // 2. 算數學插值 (start + (end - start) * t)
                                        double result = MathUtil.interpolate(startVal, endVal, t);

                                        // 3. 把 double 包回 Angle 物件
                                        return Degree.of(result);
                                });
                rollMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                (start, end, t) -> {
                                        // 邏輯：拆成 double (RPM) -> 算數學 -> 包回 Unit
                                        double startVal = start.in(RotationsPerSecond);
                                        double endVal = end.in(RotationsPerSecond);
                                        double interpolated = MathUtil.interpolate(startVal, endVal, t);
                                        return RotationsPerSecond.of(interpolated);
                                });
                ToAillancerollMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                (start, end, t) -> {
                                        // 邏輯：拆成 double (RPM) -> 算數學 -> 包回 Unit
                                        double startVal = start.in(RotationsPerSecond);
                                        double endVal = end.in(RotationsPerSecond);
                                        double interpolated = MathUtil.interpolate(startVal, endVal, t);
                                        return RotationsPerSecond.of(interpolated);
                                });
                rollMap.put(0.796222, RotationsPerSecond.of(33.8));
                rollMap.put(1.545207, RotationsPerSecond.of(35.3));
                rollMap.put(2.148772, RotationsPerSecond.of(37.8));
                rollMap.put(2.590749, RotationsPerSecond.of(38.3));
                rollMap.put(3.062585, RotationsPerSecond.of(40.8));
                rollMap.put(4.099106, RotationsPerSecond.of(42.8));
                rollMap.put(5.074542, RotationsPerSecond.of(48.3));

                hoodMap.put(0.796222, Degree.of(27.0));
                hoodMap.put(1.545207, Degree.of(32.0));
                hoodMap.put(2.148772, Degree.of(33.0));
                hoodMap.put(2.590749, Degree.of(33.5));
                hoodMap.put(3.062585, Degree.of(37.0));
                hoodMap.put(4.099106, Degree.of(40.0));
                hoodMap.put(5.074542, Degree.of(43.0));

                timeOfFlightMap.put(0.796222, 0.84);
                timeOfFlightMap.put(1.545207, 0.98);
                timeOfFlightMap.put(2.148772, 1.16);
                timeOfFlightMap.put(2.590749, 1.16);
                timeOfFlightMap.put(3.062585, 1.2);
                timeOfFlightMap.put(4.099106, 1.21);
                timeOfFlightMap.put(5.074542, 1.32);

                ToAillancerollMap.put(2.077073, RotationsPerSecond.of(29.0));
                ToAillancerollMap.put(3.185600, RotationsPerSecond.of(37.0));
                ToAillancerollMap.put(4.191824, RotationsPerSecond.of(42.0));
                ToAillancerollMap.put(5.258651, RotationsPerSecond.of(45.0));
                ToAillancerollMap.put(11.258651, RotationsPerSecond.of(77.0));

                ToAillancetimeOfFlightMap.put(2.077073, 0.91);
                ToAillancetimeOfFlightMap.put(3.185600, 1.19);
                ToAillancetimeOfFlightMap.put(4.191824, 1.31);
                ToAillancetimeOfFlightMap.put(5.258651, 1.44);
                ToAillancetimeOfFlightMap.put(11.258651, 3.0);

        }

        public record ShootingState(
                        Rotation2d turretFieldAngle, // 砲塔該瞄準的「場地角度」
                        Angle HoopAngle, // 用來查表的「有效距離
                        AngularVelocity FlywheelRPS) {
        }

        public ShootingState calculateShootingToHub() {
                Pose2d estimatedPose = drive.getPose2d();
                ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

                estimatedPose = estimatedPose.exp(new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

                Pose2d turretPosition = estimatedPose.transformBy(
                                new Transform2d(
                                                robotToTurret.getTranslation().toTranslation2d(),
                                                robotToTurret.getRotation().toRotation2d()));

                Translation2d target = AllianceFlipUtil.apply(siteConstants.topCenterPoint.toTranslation2d());
                double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

                ChassisSpeeds robotVelocity = drive.getFieldVelocity();
                double robotAngle = estimatedPose.getRotation().getRadians();
                if (AllianceFlipUtil.shouldFlip()) {
                        robotVelocity = new ChassisSpeeds(
                                        -robotVelocity.vxMetersPerSecond,
                                        -robotVelocity.vyMetersPerSecond,
                                        robotVelocity.omegaRadiansPerSecond);
                }

                // 2. 計算 Turret 的場地速度
                Translation2d turretOffsetField = robotToTurret.getTranslation().toTranslation2d()
                                .rotateBy(estimatedPose.getRotation());

                double turretVelocityX = robotVelocity.vxMetersPerSecond
                                + robotVelocity.omegaRadiansPerSecond
                                                * (robotToTurret.getY() * Math.cos(robotAngle)
                                                                - robotToTurret.getX() * Math.sin(robotAngle));
                double turretVelocityY = robotVelocity.vyMetersPerSecond
                                + robotVelocity.omegaRadiansPerSecond
                                                * (robotToTurret.getX() * Math.cos(robotAngle)
                                                                - robotToTurret.getY() * Math.sin(robotAngle));


                double timeOfFlight = 0.0;
                Pose2d lookaheadPose = turretPosition;
                double lookaheadTurretToTargetDistance = turretToTargetDistance;

                for (int i = 0; i < 5; i++) {
                        timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);

                        double offsetX = turretVelocityX * timeOfFlight;
                        double offsetY = turretVelocityY * timeOfFlight;

                        lookaheadPose = new Pose2d(
                                        turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                                        turretPosition.getRotation());

                        lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
                }

                Translation2d vectorToTarget = target.minus(lookaheadPose.getTranslation());
                Rotation2d targetFieldAngle = vectorToTarget.getAngle();

                // 3. 最後的鏡像翻轉 (保持你原本的邏輯，用於修正靜態瞄準)
                if (AllianceFlipUtil.shouldFlip()) {
                        targetFieldAngle = Rotation2d.fromDegrees(targetFieldAngle.getDegrees() - 180.0);
                }

                // Logger.recordOutput("targetFieldAngle", targetFieldAngle);

                Logger.recordOutput("lookaheadTurretToTargetDistance", lookaheadTurretToTargetDistance);

                return new ShootingState(targetFieldAngle, hoodMap.get(lookaheadTurretToTargetDistance-0.19304),
                                rollMap.get(lookaheadTurretToTargetDistance-0.19304));
        }

        // -------------------------------------------------------------------------------------------------------------------
        public ShootingState calculateShootingToAlliance() {
                // 1. 取得基本狀態
                Pose2d estimatedPose = drive.getPose2d();
                ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

                // 2. 延遲補償 (Phase Delay Compensation)
                // 預測 "現在命令發出後，實際執行時" 機器人會在哪
                estimatedPose = estimatedPose.exp(new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

                // 3. 計算砲塔位置
                Pose2d turretPosition = estimatedPose.transformBy(
                                new Transform2d(
                                                robotToTurret.getTranslation().toTranslation2d(),
                                                robotToTurret.getRotation().toRotation2d()));

                // 4. 計算目標位置 (處理紅藍翻轉)
                Translation2d target;
                if (robotStatus.getVerticalSide() == RobotStatus.VerticalSide.TOP) {
                        target = AllianceFlipUtil.apply(siteConstants.topLeftCenterPoint.toTranslation2d());
                } else {
                        target = AllianceFlipUtil.apply(siteConstants.topRightCenterPoint.toTranslation2d());
                }
                double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

                // 5. 計算砲塔的場地速度 (Turret Field Velocity)
                ChassisSpeeds robotVelocity = drive.getFieldVelocity();
                double robotAngle = estimatedPose.getRotation().getRadians();

                if (AllianceFlipUtil.shouldFlip()) {
                        robotVelocity = new ChassisSpeeds(
                                        -robotVelocity.vxMetersPerSecond,
                                        -robotVelocity.vyMetersPerSecond,
                                        robotVelocity.omegaRadiansPerSecond);
                }

                // V_turret = V_robot + (Omega x Radius)
                // 這是為了算出機器人旋轉時，砲塔本身被甩動的速度
                double turretVelocityX = robotVelocity.vxMetersPerSecond
                                + robotVelocity.omegaRadiansPerSecond
                                                * (robotToTurret.getY() * Math.cos(robotAngle)
                                                                - robotToTurret.getX() * Math.sin(robotAngle));
                double turretVelocityY = robotVelocity.vyMetersPerSecond
                                + robotVelocity.omegaRadiansPerSecond
                                                * (robotToTurret.getX() * Math.cos(robotAngle)
                                                                - robotToTurret.getY() * Math.sin(robotAngle));

                // 6. 核心迭代運算 (Iterative Solver)
                // 找出 "Lookahead Pose" (虛擬發射點)
                double timeOfFlight = 0.0;
                Pose2d lookaheadPose = turretPosition;
                double lookaheadTurretToTargetDistance = turretToTargetDistance;

                // 跑 5 次迭代通常就足夠收斂了，不用跑到 20 次
                for (int i = 0; i < 5; i++) {
                        // 查表：根據目前的預測距離，查子彈飛多久
                        timeOfFlight = ToAillancetimeOfFlightMap.get(lookaheadTurretToTargetDistance);

                        // 計算偏移量：在飛行時間內，機器人速度會把球帶偏多少
                        double offsetX = turretVelocityX * timeOfFlight;
                        double offsetY = turretVelocityY * timeOfFlight;

                        // 更新 Lookahead Pose
                        lookaheadPose = new Pose2d(
                                        turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                                        turretPosition.getRotation());

                        // 更新距離
                        lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
                }

                Translation2d vectorToTarget = target.minus(lookaheadPose.getTranslation());
                Rotation2d targetFieldAngle = vectorToTarget.getAngle();

                if (AllianceFlipUtil.shouldFlip()) {
                        // 修正：先取出 double 做運算，再轉回 Rotation2d
                        targetFieldAngle = Rotation2d.fromDegrees(targetFieldAngle.getDegrees() - 180.0);
                }

                // Logger.recordOutput("lookaheadTurretToTargetDistance",
                // lookaheadTurretToTargetDistance);
                return new ShootingState(targetFieldAngle, Hood_MAX_RADS,
                                ToAillancerollMap.get(lookaheadTurretToTargetDistance));
        }

}