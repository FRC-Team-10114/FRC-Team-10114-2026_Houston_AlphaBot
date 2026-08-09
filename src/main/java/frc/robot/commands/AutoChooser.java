package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.opencv.ml.RTrees;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.RobotStatus.RobotStatus;

public class AutoChooser {
    private final CommandSwerveDrivetrain drive;
    private final superstructure superstructure;

    public final SendableChooser<AutoStart> AutoStartChooser = new SendableChooser<>();

    public AutoChooser(CommandSwerveDrivetrain drive, superstructure superstructure) {
        this.drive = drive;
        this.superstructure = superstructure;

        this.configureAutoChoosers();
        this.SetNamedCommands();
    }

    public void SetNamedCommands() {
        NamedCommands.registerCommand("shoot", superstructure.autoshoot());
        NamedCommands.registerCommand("stopshoot", superstructure.stopShoot());
        NamedCommands.registerCommand("intake", superstructure.intake());
        NamedCommands.registerCommand("stopintake", superstructure.intakestop());
        NamedCommands.registerCommand("keepSafe", superstructure.keepsafe());
    }

    public enum AutoStart {
        LEFT, RIGHT, center, NONE
    }

    public void configureAutoChoosers() {

        AutoStartChooser.setDefaultOption("None", AutoStart.NONE);
        AutoStartChooser.addOption("Start: Left", AutoStart.LEFT);
        AutoStartChooser.addOption("Start: Right", AutoStart.RIGHT);
        AutoStartChooser.addOption("Start: Center", AutoStart.center);

        SmartDashboard.putData("Auto/Start Position", AutoStartChooser);
    }

    public Command auto() {
        AutoStart startPose = AutoStartChooser.getSelected();

        // 1. 增加 Null 防護，避免儀表板未同步導致 Crash
        if (startPose == null)
            startPose = AutoStart.NONE;

        if (startPose == AutoStart.NONE) {

            Pose2d currentPose = this.drive.getPose2d();

            try {
                Pose2d leftStart = PathPlannerPath.fromChoreoTrajectory("Left_Start")
                        .getStartingHolonomicPose()
                        .orElse(new Pose2d());

                Pose2d centerStart = PathPlannerPath.fromChoreoTrajectory("Center")
                        .getStartingHolonomicPose()
                        .orElse(new Pose2d());

                Pose2d rightStart = PathPlannerPath.fromChoreoTrajectory("RIght_Start")
                        .getStartingHolonomicPose()
                        .orElse(new Pose2d());

                // C. 計算距離 (使用 getTranslation().getDistance())
                double distLeft = currentPose.getTranslation().getDistance(leftStart.getTranslation());
                double distCenter = currentPose.getTranslation().getDistance(centerStart.getTranslation());
                double distRight = currentPose.getTranslation().getDistance(rightStart.getTranslation());

                // D. 比較誰最近
                if (distLeft < distCenter && distLeft < distRight) {
                    startPose = AutoStart.LEFT;
                } else if (distRight < distCenter && distRight < distLeft) {
                    startPose = AutoStart.RIGHT;
                } else {
                    startPose = AutoStart.center;
                }

            } catch (Exception e) {
                e.printStackTrace();
                startPose = AutoStart.center;
            }
        }

        // --- 組合路徑邏輯 ---
        Command start = Commands.none();
        switch (startPose) {
            case LEFT:
                start = new PathPlannerAuto("Left");
                break;
            case RIGHT:
                start = new PathPlannerAuto("Right");
                break;
            default:
            start = new PathPlannerAuto("Center");
                break;
        }
        return Commands.sequence(start);
    }
}