package frc.robot.util.RobotStatus;

import java.util.ArrayList;
import java.util.EventObject;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.siteConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.FIeldHelper.AllianceFlipUtil;
import frc.robot.util.RobotEvent.Event.*;

public class RobotStatus extends SubsystemBase {

    public EventObject eventObject = new EventObject(getClass());

    private static final double BLUE_ZONE_LIMIT = 5.50;
    private static final double RED_ZONE_START = FieldConstants.fieldLength - 5.50;
    private static final double MID_Y = FieldConstants.fieldWidth / 2.0;

    public final CommandSwerveDrivetrain drive;

    private final List<NeedResetPoseEvent> needResetPoseEvents = new ArrayList<>();
    private final List<InTrench> inTrenchEvents = new ArrayList<>();
    private final List<NotInTrench> notInTrenchEvents = new ArrayList<>();

    public enum Area {
        CENTER,
        BlueAlliance,
        RedAlliance
    }

    public enum VerticalSide {
        TOP,
        BOTTOM
    }

    public boolean NeedResetPose = false;

    private boolean m_wasClimbing = false;

    public boolean SafeHood = false;

    // 新增：用於狀態改變檢測 (Edge Detection)
    private boolean m_lastInTrench = false;
    // 新增：用於降低 Log 頻率
    private int logCounter = 0;

    public RobotStatus(CommandSwerveDrivetrain drive) {
        this.drive = drive;
    }

    public VerticalSide getVerticalSide() {
        double Y = drive.getPose2d().getY();
        return (Y > MID_Y) ? VerticalSide.TOP : VerticalSide.BOTTOM;
    }

    public Area getArea() {
        double x = drive.getPose2d().getX();
        if (x < BLUE_ZONE_LIMIT)
            return Area.BlueAlliance;
        if (x > RED_ZONE_START)
            return Area.RedAlliance;
        return Area.CENTER;
    }
    public void SetSafeHood(boolean bool){
        this.SafeHood = bool;
    }

    // --- 事件註冊區 ---
    public void TriggerNeedResetPoseEvent(NeedResetPoseEvent event) {
        needResetPoseEvents.add(event);
    }

    public void TriggerInTrench(InTrench event) {
        inTrenchEvents.add(event);
    }

    public void TriggerNotInTrench(NotInTrench event) {
        notInTrenchEvents.add(event);
    }

    /**
     * 核心邏輯：處理爬升後的 Pose 重置
     */
    public void updateOdometerStatus() {
        boolean isNowClimbing = this.drive.isClimbing();

        if (m_wasClimbing && !isNowClimbing) {
            for (NeedResetPoseEvent listener : needResetPoseEvents) {
                        listener.NeedResetPose();
                    }
                }
                m_wasClimbing = isNowClimbing;
            }

    /**
     * 判斷是否在我方聯盟區域
     * (已包含 Null Safety)
     */
    public boolean isInMyAllianceZone() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isEmpty())
            return false;

        Area currentArea = getArea();
        if (ally.get() == Alliance.Blue) {
            return currentArea == Area.BlueAlliance;
        } else {
            return currentArea == Area.RedAlliance;
        }
    }

    /**
     * *** 效能修復核心 ***
     * 只在狀態改變時觸發事件，而不是每個 Loop 都觸發。
     */
    public void updateTrenchStatus() {
        boolean isNowInTrench = isInTrench();

        // 只有當狀態跟上一次不一樣時 (State Changed)，才執行動作
        if (isNowInTrench != m_lastInTrench) {
            if (isNowInTrench) {
                // 剛進入 Trench
                for (InTrench listener : inTrenchEvents) {
                    listener.InTrench();
                }
            } else {
                // 剛離開 Trench
                for (NotInTrench listener : notInTrenchEvents) {
                    listener.NotInTrench();
                }
            }
            // 狀態改變時，強制記錄 Log
            Logger.recordOutput("RobotStatus/InTrench", isNowInTrench);
        }

        m_lastInTrench = isNowInTrench;
    }

    public boolean isInTrench() {
        // 如果 debug 開啟，直接回傳 true
        if (SafeHood)
            return true;

        var currentPose = drive.getPose2d();

        // 檢查一般區域
        if (isInside(currentPose, siteConstants.Right_TRENCHE_Pose1, siteConstants.Right_TRENCHE_Pose2,
                siteConstants.Right_TRENCHE_Pose3))
            return true;
        if (isInside(currentPose, siteConstants.Left_TRENCHE_Pose1, siteConstants.Left_TRENCHE_Pose2,
                siteConstants.Left_TRENCHE_Pose3))
            return true;

        // 檢查翻轉區域 (注意：AllianceFlipUtil 如果很耗時，這裡會變慢，但如果是簡單數學運算則沒問題)
        if (isInside(currentPose,
                AllianceFlipUtil.Needapply(siteConstants.Right_TRENCHE_Pose1),
                AllianceFlipUtil.Needapply(siteConstants.Right_TRENCHE_Pose2),
                AllianceFlipUtil.Needapply(siteConstants.Right_TRENCHE_Pose3)))
            return true;

        if (isInside(currentPose,
                AllianceFlipUtil.Needapply(siteConstants.Left_TRENCHE_Pose1),
                AllianceFlipUtil.Needapply(siteConstants.Left_TRENCHE_Pose2),
                AllianceFlipUtil.Needapply(siteConstants.Left_TRENCHE_Pose3)))
            return true;

        return false;
    }

    private boolean isInside(Pose2d robotPose, Pose2d... corners) {
        if (corners == null || corners.length == 0)
            return false;

        double x = robotPose.getX();
        double y = robotPose.getY();

        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;

        for (Pose2d corner : corners) {
            if (corner != null) {
                if (corner.getX() < minX)
                    minX = corner.getX();
                if (corner.getX() > maxX)
                    maxX = corner.getX();
                if (corner.getY() < minY)
                    minY = corner.getY();
                if (corner.getY() > maxY)
                    maxY = corner.getY();
            }
        }
        return (x >= minX && x <= maxX) && (y >= minY && y <= maxY);
    }

    @Override
    public void periodic() {
        // 1. 更新狀態與事件觸發
        this.updateOdometerStatus();
        this.updateTrenchStatus(); // 改名後的 trench 事件處理

        // 2. 優化 Logging：每 10 個 Loop (0.2秒) 更新一次，或是在狀態改變時更新
        // 這樣可以把 22ms 降到 < 1ms
        logCounter++;
        if (logCounter >= 25) { // 每 0.5 秒更新一次狀態 Log
            Logger.recordOutput("RobotStatus/InMyAllianceZone", isInMyAllianceZone());
            Logger.recordOutput("RobotStatus/Area", getArea().toString());
            Logger.recordOutput("RobotStatus/VerticalSide", getVerticalSide().toString());
            logCounter = 0;
        }
    }
}