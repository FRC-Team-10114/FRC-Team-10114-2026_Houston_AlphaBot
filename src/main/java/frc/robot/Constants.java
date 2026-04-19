package frc.robot;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class FieldConstants {
        private static final AprilTagFieldLayout layout;
        public static final double fieldLength;
        public static final double fieldWidth;
        static {
            try {
                // 載入預設場地 (例如 2025 Reefscape 或 2026)
                layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            } catch (Exception e) {
                throw new RuntimeException("地圖載入失敗", e);
            }
        }
        static {
            AprilTagFieldLayout layout;
            try {
                // 自動載入當年度的預設場地 (例如 2026 場地)
                layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            } catch (Exception e) {
                // 萬一讀不到檔案 (極少發生)，給個預設值防止程式崩潰
                // 這裡可以填入規則書上的大約數值
                layout = null;
                e.printStackTrace();
            }

            if (layout != null) {
                // 從官方資料直接抓取精確數值
                fieldLength = layout.getFieldLength();
                fieldWidth = layout.getFieldWidth();
            } else {
                // Fallback (保底數值)
                fieldLength = 16.54;
                fieldWidth = 8.21;
            }
        }

        public class siteConstants {
            // Dimensions
            public static final double width = Units.inchesToMeters(31.8);
            public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
            public static final double height = Units.inchesToMeters(7.0);
            public static final double bumpers = Units.inchesToMeters(73.0);
            public static final double hub = Units.inchesToMeters(47.0);

            public static final double TRENCHWide = Units.inchesToMeters(65.65);
            public static final double TRENCHdeep = Units.inchesToMeters(47.0);
            public static final double HUB_distance_to_the_ALLIANCE_WALL = Units.inchesToMeters(158.6);
            public static final double Swerve = Units.inchesToMeters(27/2 + 27);

            public static final Translation3d topCenterPoint = new Translation3d(
                    layout.getTagPose(26).get().getX() + Units.inchesToMeters(60) / 2.0,
                    fieldWidth / 2.0, // Y 軸置中
                    height // 高度固定
            );
            public static final Translation3d topLeftCenterPoint = new Translation3d(
                    layout.getTagPose(26).get().getX() + width / 2.0,
                    (fieldWidth / 2.0) + (bumpers / 2 + hub / 2), // Y 軸置中
                    height // 高度固定
            );
            public static final Translation3d topRightCenterPoint = new Translation3d(
                    layout.getTagPose(26).get().getX() + width / 2.0,
                    (fieldWidth / 2.0) - (bumpers / 2 + hub / 2), // Y 軸置中
                    height // 高度固定
            );

            public static final Pose2d Right_TRENCHE_Pose1 = new Pose2d(HUB_distance_to_the_ALLIANCE_WALL - Swerve, 0.0,
                    new Rotation2d(0.0));
            public static final Pose2d Right_TRENCHE_Pose2 = new Pose2d(HUB_distance_to_the_ALLIANCE_WALL + TRENCHWide + Swerve,
                    0.0, new Rotation2d(0.0));
            public static final Pose2d Right_TRENCHE_Pose3 = new Pose2d(HUB_distance_to_the_ALLIANCE_WALL - Swerve, TRENCHdeep,
                    new Rotation2d(0.0));
            public static final Pose2d Right_TRENCHE_Pose4 = new Pose2d(HUB_distance_to_the_ALLIANCE_WALL + TRENCHWide + Swerve,
                    TRENCHdeep,
                    new Rotation2d(0.0));

            public static final Pose2d Left_TRENCHE_Pose1 = new Pose2d(HUB_distance_to_the_ALLIANCE_WALL - Swerve,
                    FieldConstants.fieldWidth, new Rotation2d(0.0));
            public static final Pose2d Left_TRENCHE_Pose2 = new Pose2d(HUB_distance_to_the_ALLIANCE_WALL + TRENCHWide + Swerve,
                    FieldConstants.fieldWidth, new Rotation2d(0.0));
            public static final Pose2d Left_TRENCHE_Pose3 = new Pose2d(HUB_distance_to_the_ALLIANCE_WALL - Swerve,
                    FieldConstants.fieldWidth - TRENCHdeep,
                    new Rotation2d(0.0));
            public static final Pose2d Left_TRENCHE_Pose4 = new Pose2d(HUB_distance_to_the_ALLIANCE_WALL + TRENCHWide + Swerve,
                    FieldConstants.fieldWidth - TRENCHdeep,
                    new Rotation2d(0.0));
        }
    }

    public static final class SwerveModuleConstants {
        public static final String[] ModuleName = {
                "ForntLeft",
                "FrontRight",
                "BackLeft",
                "BackRight"
        };
    }

    public static final class LimelightConstants {
        public static final double MAX_GYRO_RATE = 1080;
    }

    public static final class PhotonVisionConstants {

        public static final Map<String, Transform3d> cameraTransforms = Map.of(
                "FrontRight", new Transform3d(
                        //右側
                        new Translation3d(0.3113271, -0.3113278, 0.1838034),
                        new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(-45.0))),
                "FrontLeft", new Transform3d(
                        //左側
                        new Translation3d(0.3113271, 0.3113278, 0.1838034),
                        new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(45.0))),
                    "BackRight", new Transform3d(
                        //左側
                        new Translation3d(0.1188632, -0.2648464, 0.3113339),
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-25.0), Units.degreesToRadians(-180.0))),
                    "BackLeft", new Transform3d(
                        //左側
                        new Translation3d(0.1188632, 0.2648464, 0.3113339),
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-25.0), Units.degreesToRadians(-180.0))));
                    

        public static final double borderPixels = 15.0; // 拒絕貼邊緣的角點（避免畸變/遮擋）
        public static final double maxSingleTagDistanceMeters = Units.feetToMeters(10); // 單tag最遠可接受距離
        public static final double maxYawRate = 720.0;// 最大可以接受的旋轉速度
        public static final double maxZ = 0.5; //最大高度
    }
        public static final class IDs {

        public static final class Shooter {
            public static final int FLYWHEEL_MOTOR = 15;
            public static final int HOOD_MOTOR = 25;
            public static final int TURRET_MOTOR = 20;

            public static final int HOOD_CANCODER = 55;
            public static final int TURRET_Cancoder = 21;
        }

        public static final class Intake {
            public static final int ARM_MOTOR = 13;
            public static final int Arm_follow = 14;
            public static final int ROLLER_MOTOR = 23;

            public static final int ARM_CANCODER = 15;
        }

        public static final class Hopper {
            public static final int TRIGGER_MOTOR = 30;
            public static final int SPINDEXER_MOTOR = 31;
        }

        public static final class Climber {
            public static final int CLIMBER_MOTOR = 40;
            public static final int CLIMBER_SLAVE_MOTOR = 41;
        }

        public static final class LED {
            public static final int CANDLE = 45;
        }
    }
}
