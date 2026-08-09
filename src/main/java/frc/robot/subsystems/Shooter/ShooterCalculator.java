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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
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
                // 1. 取得底盤目前的 Pose，並加上你的延遲補償 (Phase Delay) - 這是一個很棒的做法，必須保留！
                Pose2d estimatedPose = drive.getPose2d();
                ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

                estimatedPose = estimatedPose.exp(new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

                // 2. 取得場地速度 (Field-Relative Velocity)
                ChassisSpeeds fieldVelocitySpeeds = drive.getFieldVelocity();
                Translation2d fieldVelocity = new Translation2d(
                                fieldVelocitySpeeds.vxMetersPerSecond,
                                fieldVelocitySpeeds.vyMetersPerSecond);

                // 🚨 修正速度向量方向：確保速度向量與場地絕對座標系一致
                if (AllianceFlipUtil.shouldFlip()) {
                        fieldVelocity = new Translation2d(-fieldVelocity.getX(), -fieldVelocity.getY());
                }

                // 3. 取得目標的 3D 座標
                Translation2d target2d = AllianceFlipUtil.apply(siteConstants.topCenterPoint.toTranslation2d());
                // 注意：這裡的 1.8288 公尺是根據 ShotSolver 內部的 TARGET_HEIGHT (約 6 呎)。
                // 如果你的 siteConstants 有明確的 Z 軸高度，請替換掉 1.8288。
                Translation3d hubCenter3d = new Translation3d(target2d.getX(), target2d.getY(), 1.8288);

                // 4. 將 Pose2d 轉換為 ShotSolver 需要的 Pose3d
                Pose3d robotPose3d = new Pose3d(estimatedPose);

                // 5. ✨ 呼叫 ShotSolver 進行物理模擬解算 (完美取代原本的 5 次迴圈)
                ShotSolver.Solution solution = ShotSolver.solve(
                                robotPose3d,
                                fieldVelocity,
                                fieldVelocitySpeeds.omegaRadiansPerSecond,
                                hubCenter3d,
                                ShotSolver.Calibration.identity() // 使用預設微調 (無偏差)
                );

                // 6. 處理結果並轉換回 ShootingState
                if (solution.valid()) {
                        // 💡 關鍵：ShotSolver 算出的 turretYawRad 是「相對於機器人車頭的角度 (Robot-Relative)」
                        // 因為你是用底盤 (Swerve) 來瞄準，底盤的絕對目標角度 = 機器人現在角度 + 需要補償的偏移角
                        Rotation2d targetFieldAngle = estimatedPose.getRotation()
                                        .plus(Rotation2d.fromRadians(solution.turretYawRad()));

                        // 保留你原本的鏡像翻轉邏輯 (視你的底盤角度定義而定)
                        if (AllianceFlipUtil.shouldFlip()) {
                                targetFieldAngle = Rotation2d.fromDegrees(targetFieldAngle.getDegrees() - 180.0);
                        }

                        // 記錄除錯數據
                        Logger.recordOutput("ShotSolver/ToF", solution.tofSec());
                        Logger.recordOutput("ShotSolver/ExitSpeedMps", solution.exitSpeedMps());

                        // 回傳最新的 ShootingState，使用 WPILib 2024+ 的單位類別
                        return new ShootingState(
                                        targetFieldAngle,
                                        Degree.of(Math.toDegrees(solution.hoodRad())),
                                        RotationsPerSecond.of(solution.flywheelRps()));

                } else {
                        // 🚨 解算失敗防呆 (例如超出物理極限、目標太遠等)
                        // 這裡可以選擇回傳「原本舊版靜態 Map 的運算結果」作為備案，或者回傳安全的待機狀態
                        System.err.println("ShotSolver 判定無法命中 (超出射程或物理極限)!");

                        // 這裡暫時示範回傳一個靜止狀態，你可以根據需求改回讀取靜態 Map 作為 Fallback
                        return new ShootingState(
                                        estimatedPose.getRotation(),
                                        Degree.of(ShooterConstants.HARD_MIN_LIMIT),
                                        RotationsPerSecond.of(0));
                }
        }

        // -------------------------------------------------------------------------------------------------------------------
        public ShootingState calculateShootingToAlliance() {
                // 1. 取得基本狀態與延遲補償 (Phase Delay Compensation)
                Pose2d estimatedPose = drive.getPose2d();
                ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

                estimatedPose = estimatedPose.exp(new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

                // 2. 取得場地速度 (Field-Relative Velocity)
                ChassisSpeeds fieldVelocitySpeeds = drive.getFieldVelocity();
                Translation2d fieldVelocity = new Translation2d(
                                fieldVelocitySpeeds.vxMetersPerSecond,
                                fieldVelocitySpeeds.vyMetersPerSecond);

                // 🚨 速度向量紅藍方修正：確保速度向量與場地絕對座標系一致
                if (AllianceFlipUtil.shouldFlip()) {
                        fieldVelocity = new Translation2d(-fieldVelocity.getX(), -fieldVelocity.getY());
                }

                // 3. 計算目標位置 (處理紅藍翻轉與 3D 高度)
                Translation2d target2d;
                if (robotStatus.getVerticalSide() == RobotStatus.VerticalSide.TOP) {
                        target2d = AllianceFlipUtil.apply(siteConstants.topLeftCenterPoint.toTranslation2d());
                } else {
                        target2d = AllianceFlipUtil.apply(siteConstants.topRightCenterPoint.toTranslation2d());
                }

                // ✨ 設定傳球的目標高度 (Z軸)。
                // 如果傳球不需要特定的高拋物線，或者只是想把球丟給隊友，你可以自定義這裡的高度
                // 例如 0.5 公尺，或是維持預設的 1.83 (視同 Speaker 高度)
                Translation3d target3d = new Translation3d(target2d.getX(), target2d.getY(), 1.83);

                // 4. 將 Pose2d 轉換為 ShotSolver 需要的 Pose3d
                Pose3d robotPose3d = new Pose3d(estimatedPose);

                // 5. ✨ 呼叫 ShotSolver 進行物理模擬解算 (取代手動 5 次迭代)
                ShotSolver.Solution solution = ShotSolver.solve(
                                robotPose3d,
                                fieldVelocity,
                                fieldVelocitySpeeds.omegaRadiansPerSecond,
                                target3d,
                                ShotSolver.Calibration.identity());

                if (solution.valid()) {
                        // 💡 將 Robot-Relative 角度轉換為絕對場地角度
                        Rotation2d targetFieldAngle = estimatedPose.getRotation()
                                        .plus(Rotation2d.fromRadians(solution.turretYawRad()));

                        // 鏡像翻轉邏輯 (視你的底盤角度定義而定)
                        if (AllianceFlipUtil.shouldFlip()) {
                                targetFieldAngle = Rotation2d.fromDegrees(targetFieldAngle.getDegrees() - 180.0);
                        }

                        // 記錄除錯數據
                        Logger.recordOutput("ShotSolver_Alliance/ToF", solution.tofSec());
                        Logger.recordOutput("ShotSolver_Alliance/ExitSpeedMps", solution.exitSpeedMps());

                        // ✨ 注意這裡的 Hood 角度：
                        // 你原本是寫死回傳 Hood_MAX_RADS。如果你想繼續鎖死最高仰角，可以維持原樣。
                        // 但如果傳球的仰角也希望由 ShotSolver 控制，你可以改成 Math.toDegrees(solution.hoodRad())
                        return new ShootingState(
                                        targetFieldAngle,
                                        Degree.of(Math.toDegrees(solution.hoodRad())), // 或者替換為:
                                                                                       // Degree.of(Math.toDegrees(solution.hoodRad()))
                                        RotationsPerSecond.of(solution.flywheelRps()));

                } else {
                        // 🚨 解算失敗防呆
                        System.err.println("ShotSolver 判定傳球無法達成 (超出物理極限)!");

                        // 這裡可以選擇回傳「靜態查表法」作為備案，或者回傳預設狀態
                        return new ShootingState(
                                        estimatedPose.getRotation(),
                                        Hood_MAX_RADS,
                                        RotationsPerSecond.of(0) // 不發射
                        );
                }
        }

}