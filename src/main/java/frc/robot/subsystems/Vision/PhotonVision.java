package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
    private AprilTagFieldLayout fieldLayout;

    // 儲存所有的相機相對轉換，讓 Simulation 也能讀取
    private final Map<String, Transform3d> m_cameraTransforms;

    private static class CamWrapper {
        final String name;
        final PhotonCamera cam;
        final PhotonPoseEstimator estimator;
        final Transform3d robotToCamera;

        CamWrapper(String name, PhotonCamera cam, PhotonPoseEstimator estimator, Transform3d robotToCamera) {
            this.name = name;
            this.cam = cam;
            this.estimator = estimator;
            this.robotToCamera = robotToCamera;
        }
    }

    private final double borderPixels = PhotonVisionConstants.borderPixels;
    private final double maxSingleTagDistanceMeters = PhotonVisionConstants.maxSingleTagDistanceMeters;
    private final List<CamWrapper> cams = new ArrayList<>();

    public PhotonVision(CommandSwerveDrivetrain drive, Map<String, Transform3d> cameraTransforms) {
        this.drivetrain = drive;
        this.m_cameraTransforms = cameraTransforms; // 儲存起來
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // 如果是模擬模式，跳過實體相機硬體初始化，但保留配置
        // if (Robot.isSimulation()) {
        // System.out.println("Simulation mode - Initializing Virtual Camera System");
        // return;
        // }

        cameraTransforms.forEach((name, transform) -> {
            PhotonCamera cam = new PhotonCamera(name);
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                    fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    transform);
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            cams.add(new CamWrapper(name, cam, estimator, transform));
        });
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) {
            runSimulationVision();
        } else {
            updateVision();
        }
    }

    /**
     * 🧪 模擬專用：多相機、全場 AprilTag 視野物理模擬
     */
    private void runSimulationVision() {
        List<Pose3d> linePoints = new ArrayList<>();
        Pose3d robotPose3d = new Pose3d(drivetrain.getState().Pose);

        // 1. 遍歷你在 RobotContainer 中傳入的所有相機 (如 FrontLeft, FrontRight)
        m_cameraTransforms.forEach((name, transform) -> {
            // 計算該相機目前在 3D 場地上的絕對位置
            Pose3d cameraPose3d = robotPose3d.transformBy(transform);

            // 2. 遍歷今年場地上的「所有」AprilTag
            for (var aprilTag : fieldLayout.getTags()) {
                Pose3d tagPose = aprilTag.pose;

                // 計算相機到 Tag 的向量與距離
                Translation3d camToTag = tagPose.getTranslation().minus(cameraPose3d.getTranslation());
                double distance = camToTag.getNorm();

                // 物理過濾 A：最大偵測距離限制（修改為 5.0 公尺）
                if (distance < 5.0) {

                    // 物理過濾 B：視角限制 (FOV Filter)
                    // 將 Tag 座標轉換為相機的局部座標系（+X為鏡頭前方，+Y為左，+Z為上）
                    Pose3d tagInCamFrame = new Pose3d(camToTag, tagPose.getRotation()).relativeTo(cameraPose3d);

                    double localX = tagInCamFrame.getX();
                    double localY = tagInCamFrame.getY();
                    double localZ = tagInCamFrame.getZ();

                    // Tag 必須在鏡頭的前方 (X > 0)
                    if (localX > 0) {
                        // 計算水平與垂直偏角
                        double horizontalAngle = Math.toDegrees(Math.atan2(localY, localX));
                        double verticalAngle = Math.toDegrees(Math.atan2(localZ, localX));

                        // 物理過濾 C：修改為你的相機視野
                        // 水平 FOV 70度：左右偏角在 35度內
                        // 垂直 FOV 依比例設定（通常為水平的 3/4，約 52.5度，亦即上下偏角在 26.25度內）
                        if (Math.abs(horizontalAngle) < 35.0 && Math.abs(verticalAngle) < 26.25) {
                            // 成功偵測！成對放入雷射線
                            linePoints.add(cameraPose3d); // 起點：該相機位置
                            linePoints.add(tagPose); // 終點：該 AprilTag 位置
                        }
                    }
                }
            }
        });

        // 輸出所有綠色雷射線
        // Logger.recordOutput("Vision/TagLines", linePoints.toArray(new Pose3d[0]));
    }

    private void updateVision() {
        // List<Pose3d> linePoints = new ArrayList<>();
        // Pose3d robotPose3d = new Pose3d(drivetrain.getState().Pose);

        for (CamWrapper cw : cams) {
            for (PhotonPipelineResult result : cw.cam.getAllUnreadResults()) {

                if (result.hasTargets()) {
                    m_lastTagId = result.getBestTarget().getFiducialId();
                    // Pose3d cameraPose3d = robotPose3d.transformBy(cw.robotToCamera);

                    for (var target : result.getTargets()) {
                        int tagId = target.getFiducialId();
                        Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);

                        // if (tagPoseOpt.isPresent()) {
                        //     Pose3d tagPose = tagPoseOpt.get();
                        //     linePoints.add(cameraPose3d);
                        //     linePoints.add(tagPose);
                        // }
                    }
                }

                Optional<EstimatedRobotPose> poseOpt = cw.estimator.update(result);
                if (poseOpt.isEmpty())
                    continue;

                EstimatedRobotPose est = poseOpt.get();
                Pose3d cameraRobotPose3d = est.estimatedPose;
                double resultTimeSec = est.timestampSeconds;

                double rotSpeed = Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
                if (rotSpeed > PhotonVisionConstants.maxYawRate)
                    continue;

                int numTags = est.targetsUsed.size();
                double avgDist = 0.0;
                for (var tgt : est.targetsUsed) {
                    avgDist += tgt.getBestCameraToTarget().getTranslation().getNorm();
                }
                if (numTags > 0)
                    avgDist /= numTags;

                if (numTags < 2) {
                    if (avgDist > maxSingleTagDistanceMeters)
                        continue;

                    if (result.getBestTarget() != null && result.getBestTarget().getPoseAmbiguity() > 0.2)
                        continue;
                }

                Vector<N3> stdDevs;
                if (this.NeedResetPose) {
                    stdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(2));
                    this.NeedResetPose = false;
                } else {
                    if (numTags >= 2) {
                        stdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(4));
                    } else {
                        double distErr = 0.5 * Math.pow(avgDist, 2);
                        stdDevs = VecBuilder.fill(distErr, distErr, 99999.0);
                    }
                }
                // Logger.recordOutput("cameraRobotPose", cameraRobotPose3d);
                drivetrain.addVisionMeasurement(
                        cameraRobotPose3d.toPose2d(),
                        resultTimeSec,
                        stdDevs);
            }
        }

        // Logger.recordOutput("Vision/TagLines", linePoints.toArray(new Pose3d[0]));
    }

    // ... 下方其餘 resetPoseToVision 等方法保持不變 ...
    public boolean resetPoseToVision() {
        Pose2d bestPose = null;
        double minScore = 99999.0;
        for (CamWrapper cw : cams) {
            PhotonPipelineResult result = cw.cam.getLatestResult();
            if (!result.hasTargets())
                continue;
            Optional<EstimatedRobotPose> poseOpt = cw.estimator.update(result);
            if (poseOpt.isEmpty())
                continue;
            EstimatedRobotPose est = poseOpt.get();
            Pose3d pose3d = est.estimatedPose;
            if (Math.abs(pose3d.getZ()) > 0.5)
                continue;
            int numTags = est.targetsUsed.size();
            double avgDist = 0.0;
            for (var tgt : est.targetsUsed) {
                avgDist += tgt.getBestCameraToTarget().getTranslation().getNorm();
            }
            if (numTags > 0)
                avgDist /= numTags;
            if (numTags == 1) {
                var bestTarget = result.getBestTarget();
                if (bestTarget != null && bestTarget.getPoseAmbiguity() > 0.2)
                    continue;
            }
            double currentScore = numTags >= 2 ? avgDist : 100.0 + avgDist;
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
            if (cw.name.equals(cameraName))
                return cw.cam.isConnected();
        }
        return false;
    }

    public boolean isAnyCameraConnected() {
        for (CamWrapper cw : cams) {
            if (cw.cam.isConnected())
                return true;
        }
        return false;
    }
}