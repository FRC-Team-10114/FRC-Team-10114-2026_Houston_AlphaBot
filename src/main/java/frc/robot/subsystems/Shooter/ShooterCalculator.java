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
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.util.FIeldHelper.AllianceFlipUtil;
import frc.robot.util.RobotStatus.RobotStatus;

public class ShooterCalculator {
        private final RobotStatus robotStatus;
        private final CommandSwerveDrivetrain drive;

        private final InterpolatingTreeMap<Double, Angle> hoodMap;
        private final InterpolatingTreeMap<Double, AngularVelocity> rollMap;
        private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

        private final double time_error = 0.0;
        private final double phaseDelay = 0.03 + time_error;
        private final double linearDragTimeConstant = 0.375;

        public static Transform3d robotToTurret = new Transform3d(0.2, 0.0, 0.44, Rotation3d.kZero);
        private static final Angle Hood_MAX_RADS = ShooterConstants.Hood_MAX_LIMIT;

        public ShooterCalculator(CommandSwerveDrivetrain drive, RobotStatus robotStatus) {
                this.drive = drive;
                this.robotStatus = robotStatus;

                hoodMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                (start, end, t) -> Degree
                                                .of(MathUtil.interpolate(start.in(Degree), end.in(Degree), t)));
                rollMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                (start, end, t) -> RotationsPerSecond.of(MathUtil.interpolate(
                                                start.in(RotationsPerSecond), end.in(RotationsPerSecond), t)));

                // ================== Hub 查表 ==================5.465927

                rollMap.put(2.109356, RotationsPerSecond.of(34.0));
                rollMap.put(2.954569, RotationsPerSecond.of(39.0));
                rollMap.put(3.2456, RotationsPerSecond.of(43.0));
                rollMap.put(3.90871, RotationsPerSecond.of(45.5));
                rollMap.put(4.644838, RotationsPerSecond.of(52.0));


                hoodMap.put(2.109356, Degree.of(24.0));
                hoodMap.put(2.954569, Degree.of(26.0));
                hoodMap.put(3.2456, Degree.of(28.0));
                hoodMap.put(3.90871, Degree.of(30.0));
                hoodMap.put(4.644838, Degree.of(32.0));

                timeOfFlightMap.put(2.109356, 1.07);
                timeOfFlightMap.put(2.954569, 1.14);
                timeOfFlightMap.put(3.2456, 1.22);
                timeOfFlightMap.put(3.90871, 1.23);
                timeOfFlightMap.put(4.644838, 1.36);


        }

        public record ShootingState(
                        Rotation2d turretRelativeAngle, // 🟢 改為相對車頭的角度
                        Angle HoopAngle,
                        AngularVelocity FlywheelRPS) {
        }

        // ========================================================================================
        // 🚀 主目標 (Hub) 射擊計算
        // ========================================================================================
        public ShootingState calculateShootingToHub() {
                Pose2d estimatedPose = drive.getPose2d();
                ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

                estimatedPose = estimatedPose.exp(new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

                Pose2d turretPosition = estimatedPose.transformBy(
                                new Transform2d(
                                                ShooterConstants.robotToTurret.getTranslation().toTranslation2d(),
                                                ShooterConstants.robotToTurret.getRotation().toRotation2d()));

                // 目標座標的紅藍翻轉
                Translation2d target = AllianceFlipUtil.apply(siteConstants.topCenterPoint.toTranslation2d());

                double currentTurretToTargetDistance = target.getDistance(turretPosition.getTranslation());
                // Logger.recordOutput("ShooterCalculator/Hub_CurrentDistanceMeters",
                // currentTurretToTargetDistance);

                ChassisSpeeds robotVelocity = drive.getFieldVelocity();
                if (AllianceFlipUtil.shouldFlip()) {
                        robotVelocity = new ChassisSpeeds(
                                        -robotVelocity.vxMetersPerSecond,
                                        -robotVelocity.vyMetersPerSecond,
                                        robotVelocity.omegaRadiansPerSecond);
                }

                Translation2d turretOffsetField = ShooterConstants.robotToTurret.getTranslation().toTranslation2d()
                                .rotateBy(estimatedPose.getRotation());

                double turretVelocityX = robotVelocity.vxMetersPerSecond
                                - (robotVelocity.omegaRadiansPerSecond * turretOffsetField.getY());
                double turretVelocityY = robotVelocity.vyMetersPerSecond
                                + (robotVelocity.omegaRadiansPerSecond * turretOffsetField.getX());

                double timeOfFlight = 0.0;
                double effectiveTOF = 0.0;
                Pose2d lookaheadPose = turretPosition;
                double lookaheadTurretToTargetDistance = currentTurretToTargetDistance;

                for (int i = 0; i < 5; i++) {
                        timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);

                        effectiveTOF = (1 - Math.exp(-timeOfFlight * linearDragTimeConstant))
                                        / linearDragTimeConstant;

                        double offsetX = turretVelocityX * effectiveTOF;
                        double offsetY = turretVelocityY * effectiveTOF;

                        lookaheadPose = new Pose2d(
                                        turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                                        turretPosition.getRotation());

                        lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
                }
                Logger.recordOutput("ShooterCalculator/Hub_LookaheadDistanceMeters", lookaheadTurretToTargetDistance);

                Translation2d vectorToTarget = target.minus(lookaheadPose.getTranslation());
                Rotation2d targetFieldAngle = vectorToTarget.getAngle();

                // 🟢 關鍵修改：將「絕對場地角度」扣除「機器人絕對角度」，轉為「相對車頭角度」
                // 這樣無論紅藍方，正前方永遠是 0 度。不需要再用 AllianceFlipUtil 對角度作翻轉了！
                Rotation2d targetRelativeAngle = targetFieldAngle.minus(estimatedPose.getRotation());

                return new ShootingState(
                                targetRelativeAngle, // 🟢 輸出為相對角度
                                hoodMap.get(lookaheadTurretToTargetDistance),
                                rollMap.get(lookaheadTurretToTargetDistance));
        }

        public ShootingState calculateShootingToAlliance() {
                Pose2d estimatedPose = drive.getPose2d();
                ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

                estimatedPose = estimatedPose.exp(new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

                Pose2d turretPosition = estimatedPose.transformBy(
                                new Transform2d(
                                                ShooterConstants.robotToTurret.getTranslation().toTranslation2d(),
                                                ShooterConstants.robotToTurret.getRotation().toRotation2d()));

                Translation2d target;
                if (robotStatus.getVerticalSide() == RobotStatus.VerticalSide.TOP) {
                        target = AllianceFlipUtil.apply(siteConstants.topLeftCenterPoint.toTranslation2d());
                } else {
                        target = AllianceFlipUtil.apply(siteConstants.topRightCenterPoint.toTranslation2d());
                }

                double currentTurretToTargetDistance = target.getDistance(turretPosition.getTranslation());
                // Logger.recordOutput("ShooterCalculator/Alliance_CurrentDistanceMeters", currentTu。rretToTargetDistance);

                ChassisSpeeds robotVelocity = drive.getFieldVelocity();
                if (AllianceFlipUtil.shouldFlip()) {
                        robotVelocity = new ChassisSpeeds(
                                        -robotVelocity.vxMetersPerSecond,
                                        -robotVelocity.vyMetersPerSecond,
                                        robotVelocity.omegaRadiansPerSecond);
                }

                Translation2d turretOffsetField = ShooterConstants.robotToTurret.getTranslation().toTranslation2d()
                                .rotateBy(estimatedPose.getRotation());

                double turretVelocityX = robotVelocity.vxMetersPerSecond
                                - (robotVelocity.omegaRadiansPerSecond * turretOffsetField.getY());
                double turretVelocityY = robotVelocity.vyMetersPerSecond
                                + (robotVelocity.omegaRadiansPerSecond * turretOffsetField.getX());

                double timeOfFlight = 0.0;
                double effectiveTOF = 0.0;
                Pose2d lookaheadPose = turretPosition;
                double lookaheadTurretToTargetDistance = currentTurretToTargetDistance;

                for (int i = 0; i < 5; i++) {
                        timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);

                        effectiveTOF = (1 - Math.exp(-timeOfFlight * linearDragTimeConstant))
                                        / linearDragTimeConstant;

                        double offsetX = turretVelocityX * effectiveTOF;
                        double offsetY = turretVelocityY * effectiveTOF;

                        lookaheadPose = new Pose2d(
                                        turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                                        turretPosition.getRotation());

                        lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
                }
                // Logger.recordOutput("ShooterCalculator/Alliance_LookaheadDistanceMeters",
                                // lookaheadTurretToTargetDistance);

                Translation2d vectorToTarget = target.minus(lookaheadPose.getTranslation());
                Rotation2d targetFieldAngle = vectorToTarget.getAngle();

                // 🟢 將「絕對場地角度」扣除「機器人絕對角度」，轉為「相對車頭角度」
                Rotation2d targetRelativeAngle = targetFieldAngle.minus(estimatedPose.getRotation());

                return new ShootingState(
                                targetRelativeAngle, // 🟢 輸出為相對角度
                                hoodMap.get(lookaheadTurretToTargetDistance),
                                rollMap.get(lookaheadTurretToTargetDistance));
        }

}