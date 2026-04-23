// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IDs.LED;
import frc.robot.subsystems.superstructure;
import frc.robot.subsystems.Dashboard.Dashboard;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainTest;
import frc.robot.subsystems.Drivetrain.TunerConstants;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.PhotonVision;
import frc.robot.util.FMS.Signal;
import frc.robot.util.RobotStatus.RobotStatus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class RobotContainer {
  // Constants for tuning
  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxTeleOpSpeed = MaxSpeed;
  private final double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxTeleOpSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController controller = new CommandXboxController(1);

  final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final SwerveDrivetrainTest[] tests = new SwerveDrivetrainTest[4];
  private final Telemetry logger = new Telemetry(this.drivetrain.getState());
  private final RobotStatus robotStatus = new RobotStatus(drivetrain);

  // public final PhotonVision photonVision = new PhotonVision(drivetrain,
  // Constants.PhotonVisionConstants.cameraTransforms);

  private final Field2d field = new Field2d();

  private final ShooterSubsystem shooter = ShooterSubsystem.create(drivetrain, robotStatus);
  private final IntakeSubsystem intake = IntakeSubsystem.create();
  private final HopperSubsystem hopper = HopperSubsystem.create();

  private final LED led = new LED();

  private final superstructure superstructure = new superstructure(shooter, intake, hopper);

  public final Signal signal = new Signal();

  private final Dashboard dashboard = new Dashboard(signal);

  public RobotContainer() {
    // Swerve Drivetrain Current Test
    for (int i = 0; i < 4; i++)
      this.tests[i] = new SwerveDrivetrainTest(drivetrain, i);

    configureBindings();

    configureEvents();

    log();

    configureBindings();
  }

  private void configureEvents() {
    // robotStatus.TriggerNeedResetPoseEvent(photonVision::NeedResetPoseEvent);
    robotStatus.TriggerInTrench(shooter::TrueInTrench);
    robotStatus.TriggerNotInTrench(shooter::FalsInTrench);
    signal.TargetInactive(shooter::FalseTargetactive);
    signal.Targetactive(shooter::TrueTargetactive);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
          return drive

              .withVelocityX(-joystick.getLeftY() * MaxTeleOpSpeed)

              .withVelocityY(-joystick.getLeftX() * MaxTeleOpSpeed)

              .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
        }));
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.rightTrigger().whileTrue(this.superstructure.shootCommand())
        .onFalse(this.superstructure.stopShoot());

    joystick.leftTrigger()
        .whileTrue(Commands.run(superstructure::intake, superstructure))
        .onFalse(Commands.runOnce(superstructure::intakestop, superstructure));
  }

  public void log() {
    SmartDashboard.putData("Field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("path").setPoses(poses);
    });
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
