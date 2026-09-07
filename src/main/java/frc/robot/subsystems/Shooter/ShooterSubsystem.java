package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.google.gson.annotations.Until;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter.ShooterCalculator.ShootingState;
import frc.robot.subsystems.Shooter.Trigger.TriggerIO;
import frc.robot.subsystems.Shooter.Trigger.TriggerIOTalon;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelHardware;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Hood.HoodIOTalon;
import frc.robot.subsystems.Shooter.Turret.TurretHardware;
import frc.robot.subsystems.Shooter.Turret.TurretIO;
// import frc.robot.subsystems.Shooter.Turret.TurretIOSpark;
import frc.robot.subsystems.Shooter.Turret.TurretIO.ShootState;
import frc.robot.util.RobotStatus.RobotStatus;

public class ShooterSubsystem extends SubsystemBase {

    private int skipCounter = 200;

    private final HoodIO hood;
    private final FlywheelIO flywheel;
    private final TurretIO turret;
    private final ShooterCalculator shooterCalculator;
    private final CommandSwerveDrivetrain drive;
    private final TriggerIO trigger;

    private ShootState currentShootState = ShootState.TRACKING;

    private final RobotStatus robotStatus;

    private double flywheelRPS = 0.0;

    private AngularVelocity flywheelgoal;

    private boolean Targetactive = true;

    private boolean Isshooting = false;

    private boolean InTrench = false;

    public boolean cameralow = false;

    private Angle HoodtargetAngle = Degrees.of(25);

    private Angle turretAngle = Radians.of(0);

    public ShooterSubsystem(TriggerIO trigger, HoodIO hood, FlywheelIO flywheel, TurretIO turret,
            ShooterCalculator shooterCalculator,
            CommandSwerveDrivetrain drive, RobotStatus robotStatus) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.turret = turret;
        this.shooterCalculator = shooterCalculator;
        this.drive = drive;
        this.robotStatus = robotStatus;
        this.trigger = trigger;
    }

    public static ShooterSubsystem create(CommandSwerveDrivetrain drive, RobotStatus status) {
        return new ShooterSubsystem(
                new TriggerIOTalon(),
                new HoodIOTalon(),
                new FlywheelHardware(),
                new TurretHardware(),
                new ShooterCalculator(drive, status),
                drive,
                status);
    }

    @Override
    public void periodic() {
        if (this.skipCounter-- > 0) return;

        SetShooterGoal();
        Logger.recordOutput("turretangle", this.turret.getAngle());
        Logger.recordOutput("HoodtargetAngle", HoodtargetAngle);
        Logger.recordOutput("getAnglegoal", this.turret.getAnglegoal());
    }
    public void TrueIsshooting() {
        Isshooting = true;
    }

    public void FalseIsshooting() {
        Isshooting = false;
    }

    public void TrueTargetactive() {
        Targetactive = true;
    }

    public void FalseTargetactive() {
        Targetactive = false;
    }

    public void TrueInTrench() {
        InTrench = true;
    }

    public void FalsInTrench() {
        InTrench = false;
    }

    public boolean SpinAllTime() {
        if (Targetactive == true && robotStatus.isInMyAllianceZone() == true) {
            return true;
        } else {
            return false;
        }
    }

    public ShootingState shooterTargetChoose() {
        if (robotStatus.getArea() == RobotStatus.Area.CENTER) {
            return this.shooterCalculator.calculateShootingToAlliance();
        } else {
            return this.shooterCalculator.calculateShootingToHub();
        }
    }

    public void SetShooterGoal() {

        ShootingState state = this.shooterTargetChoose();

        Rotation2d targetFieldAngle = state.turretRelativeAngle();

        Angle TurretTarget = Radians.of(targetFieldAngle.getRadians());

        HoodtargetAngle = state.HoopAngle();

        AngularVelocity FlywheelRPS = state.FlywheelRPS();

        flywheelgoal = FlywheelRPS;

        // this.setHoodAngle(HoodTarget);

        this.setTurretAngle(drive.getRotation(), TurretTarget);

        Logger.recordOutput("HoodTarget", HoodtargetAngle);

        Logger.recordOutput("flywheelgoal", flywheelgoal);

        Logger.recordOutput("TurretTarget", TurretTarget);

        Logger.recordOutput("InTrench", InTrench);
    }

    public void setHoodAngle(Angle targetRad) {
        if (InTrench) {
            this.hood.setAngle(ShooterConstants.Hood_MIN_LIMIT);
        } else {
            this.hood.setAngle(targetRad);
        }
    }

    public void shoot() {
        this.setHoodAngle(HoodtargetAngle);
        this.flywheel.setRPS(flywheelgoal);
        if (isAtSetPosition()) {
            this.trigger.setRPS(RotationsPerSecond.of(65.0));
        }
    }

    public void stopShoot() {
        this.flywheel.stop();
        this.trigger.stop();
        this.hood.setAngle(ShooterConstants.Hood_MIN_LIMIT);
    }

    public void setTurretAngle(Rotation2d robotAngle, Angle targetRad) {
        this.turret.setAngle(robotAngle, targetRad, currentShootState);
    }

    public boolean isAtSetPosition() {
        return flywheel.isAtSetPosition() &&
        turret.isAtSetPosition() &&
                hood.isAtSetPosition();
    }

    public void setShootingState() {
        if (Isshooting) {
            this.currentShootState = ShootState.ACTIVE_SHOOTING;
        } else {
            this.currentShootState = ShootState.TRACKING;
        }
    }

    public Command waithoodsafe() {
        return Commands.sequence(
                Commands.runOnce(() -> this.hood.setAngle(ShooterConstants.Hood_MIN_LIMIT), this),
                Commands.waitUntil(() -> this.hood.isAtSetPosition()));
    }
}