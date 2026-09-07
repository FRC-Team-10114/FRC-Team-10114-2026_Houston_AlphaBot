package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure;

public class AutoChooser {

    private final CommandSwerveDrivetrain drive;
    private final superstructure superstructure;

    public final SendableChooser<AutoStart> AutoStartChooser =
            new SendableChooser<>();

    public enum AutoStart {
        LEFT,
        RIGHT,
        center,
        NONE
    }

    /*-------------------- 預先載入起始位置 --------------------*/

    private final Pose2d leftStartPose;
    private final Pose2d centerStartPose;
    private final Pose2d rightStartPose;

    /*-------------------- 預先建立 Auto --------------------*/

    private final Command leftAuto;
    private final Command centerAuto;
    private final Command rightAuto;

    public AutoChooser(
            CommandSwerveDrivetrain drive,
            superstructure superstructure) {

        this.drive = drive;
        this.superstructure = superstructure;

        configureAutoChoosers();

        SetNamedCommands();

        /*-------------------- 載入 Choreo --------------------*/

        leftStartPose = loadStartPose("Left_Start");
        centerStartPose = loadStartPose("Center");
        rightStartPose = loadStartPose("RIght_Start");

        /*-------------------- 建立 PathPlanner Auto --------------------*/

        leftAuto = new PathPlannerAuto("Left");
        centerAuto = new PathPlannerAuto("Center");
        rightAuto = new PathPlannerAuto("Right");
    }

    private Pose2d loadStartPose(String name) {

        try {
            return PathPlannerPath
                    .fromChoreoTrajectory(name)
                    .getStartingHolonomicPose()
                    .orElse(new Pose2d());

        } catch (Exception e) {

            e.printStackTrace();

            return new Pose2d();
        }
    }

    public void SetNamedCommands() {

        NamedCommands.registerCommand(
                "shoot",
                superstructure.autoshoot());

        NamedCommands.registerCommand(
                "stopshoot",
                superstructure.stopShoot());

        NamedCommands.registerCommand(
                "intake",
                superstructure.intake());

        NamedCommands.registerCommand(
                "stopintake",
                superstructure.intakestop());

        NamedCommands.registerCommand(
                "keepSafe",
                superstructure.keepsafe());
    }

    public void configureAutoChoosers() {

        AutoStartChooser.setDefaultOption(
                "None",
                AutoStart.NONE);

        AutoStartChooser.addOption(
                "Start: Left",
                AutoStart.LEFT);

        AutoStartChooser.addOption(
                "Start: Right",
                AutoStart.RIGHT);

        AutoStartChooser.addOption(
                "Start: Center",
                AutoStart.center);

        SmartDashboard.putData(
                "Auto/Start Position",
                AutoStartChooser);
    }

    public Command auto() {

        AutoStart startPose =
                AutoStartChooser.getSelected();

        if (startPose == null) {
            startPose = AutoStart.NONE;
        }

        if (startPose == AutoStart.NONE) {

            Pose2d currentPose =
                    drive.getPose2d();

            double distLeft =
                    currentPose
                            .getTranslation()
                            .getDistance(
                                    leftStartPose.getTranslation());

            double distCenter =
                    currentPose
                            .getTranslation()
                            .getDistance(
                                    centerStartPose.getTranslation());

            double distRight =
                    currentPose
                            .getTranslation()
                            .getDistance(
                                    rightStartPose.getTranslation());

            if (distLeft < distCenter
                    && distLeft < distRight) {

                startPose = AutoStart.LEFT;

            } else if (distRight < distCenter
                    && distRight < distLeft) {

                startPose = AutoStart.RIGHT;

            } else {

                startPose = AutoStart.center;
            }
        }

        Command start;

        switch (startPose) {

            case LEFT:
                start = leftAuto;
                break;

            case RIGHT:
                start = rightAuto;
                break;

            case center:
            default:
                start = centerAuto;
                break;
        }

        return start;
    }
}