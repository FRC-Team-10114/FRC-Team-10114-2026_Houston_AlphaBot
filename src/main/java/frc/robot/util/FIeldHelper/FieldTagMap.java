package frc.robot.util.FIeldHelper;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;

public class FieldTagMap {

    public static final Map<Integer, Pose3d> tagMap = new HashMap<>();
    private static final int[] LEFT_TRENCH_IDS = { 22, 23 };
    private static final int[] RIGHT_TRENCH_IDS = { 17, 28 };
    private static final double HALF_WIDTH = 1.668 / 2.0;


    static{loadTags();}

    private static void loadTags() {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        for (AprilTag tag : layout.getTags()) {
            tagMap.put(tag.ID, tag.pose);
        }
    }

    public static Pose3d getPose3d(int id) {
        return tagMap.get(id);
    }

    public static Pose2d getPose2d(int id) {
        Pose3d pose3d = tagMap.get(id);
        if (pose3d == null)
            return null;
        return pose3d.toPose2d();
    }

    public static boolean hasTag(int id) {
        return tagMap.containsKey(id);
    }
    public static Pose2d getLeftTrenchPose() {
        return calculateAveragePose(LEFT_TRENCH_IDS);
    }
    public static Pose2d getRightTrenchPose() {
        return calculateAveragePose(RIGHT_TRENCH_IDS);
    }
    private static Pose2d calculateAveragePose(int[] ids) {
        Pose2d pose1 = FieldTagMap.getPose2d(ids[0]);
        Pose2d pose2 = FieldTagMap.getPose2d(ids[1]);

        double avgX = (pose1.getX() + pose2.getX()) / 2.0;
        double avgY = (pose1.getY() + pose2.getY()) / 2.0;

        double avgCos = pose1.getRotation().getCos() + pose2.getRotation().getCos();
        double avgSin = pose1.getRotation().getSin() + pose2.getRotation().getSin();

        Rotation2d avgRot = new Rotation2d(avgCos, avgSin);

        return new Pose2d(avgX, avgY, avgRot);
    }
    public static Pose2d[] getLeftTrenchPoses() {
        Pose2d center = calculateAveragePose(LEFT_TRENCH_IDS);
        return applyXOffset(center);
    }
    public static Pose2d[] getRightTrenchPoses() {
        Pose2d center = calculateAveragePose(RIGHT_TRENCH_IDS);
        return applyXOffset(center);
    }

    private static Pose2d[] applyXOffset(Pose2d center) {
        Pose2d poseMinus = new Pose2d(
            center.getX() - HALF_WIDTH, 
            center.getY(), 
            center.getRotation()
        );
        Pose2d posePlus = new Pose2d(
            center.getX() + HALF_WIDTH, 
            center.getY(), 
            center.getRotation()
        );

        return new Pose2d[] { poseMinus, posePlus };
    }
}