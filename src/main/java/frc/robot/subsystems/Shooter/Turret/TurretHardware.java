package frc.robot.subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.logging.Logger;

import org.opencv.core.Mat;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class TurretHardware extends TurretIO {
    private final TalonFX turretMotor;
    private final CANcoder turretCaNcoder;
    private final StatusSignal<Angle> turretPosition;

    private final double gearRatio = (96.0 / 16.0) * 3.0;

    private final DynamicMotionMagicVoltage m_request = new DynamicMotionMagicVoltage(0, 0, 0);
    private final VoltageOut voltagRequire = new VoltageOut(0.0);
    public Angle goal;

    private final SysIdRoutine sysIdRoutine;

    private final double BASE_VELOCITY = 1.5;

    private final double BASE_ACCELERATION = 6.0;

    private final double BASE_JERK = 80.0 / (2.0 * Math.PI);

    public TurretHardware() {
        this.turretMotor = new TalonFX(IDs.Shooter.TURRET_MOTOR, "canivore");
        this.turretCaNcoder = new CANcoder(IDs.Shooter.TURRET_Cancoder, "canivore");
        this.turretPosition = turretMotor.getPosition();

        SignalLogger.setPath("/U/");

        this.sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(3),
                        null, (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> this.turretMotor.setControl(voltagRequire.withOutput(volts.in(Volts))),
                        null,
                        new SubsystemBase() {
                            @Override
                            public String getName() {
                                return "TurretSysId";
                            }
                        }));

        this.CANcoderConfig();
        configureMotors();
        seedPosition();
    }

    public void CANcoderConfig() {
        var cfg = new CANcoderConfiguration();
        cfg.MagnetSensor.MagnetOffset = -0.14697265625;
        cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        turretCaNcoder.getConfigurator().apply(cfg);
    }

    public void configureMotors() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0);

        configs.SoftwareLimitSwitch
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(ShooterConstants.HARD_MIN_LIMIT)
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(ShooterConstants.HARD_MAX_LIMIT);

        configs.Feedback
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(18.0);

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        configs.Slot0.kS = 0.63542;

        configs.Slot0.kV = 1.5255 * 2 * Math.PI;
        configs.Slot0.kA = 0.13204 * 2 * Math.PI;

        configs.Slot0.kP = 42.0;
        configs.Slot0.kD = 0.5;

        turretMotor.getConfigurator().apply(configs);
    }

    public void seedPosition() {
        turretCaNcoder.getAbsolutePosition().waitForUpdate(0.250);

        double cancoderRotations = turretCaNcoder.getAbsolutePosition().getValueAsDouble();

        double mechanismRotations = cancoderRotations * 2.0;

        turretMotor.setPosition(mechanismRotations);
    }

    @Override
    public void setAngle(Rotation2d robotHeading, Angle targetRad, ShootState state) {
        double target = calculate(robotHeading, targetRad, state).in(Radians);
        double current = getAngle().in(Radians);
        double error = target - current;

        double currentMaxVel = BASE_VELOCITY;
        double currentMaxAccel = BASE_ACCELERATION;
        double extraFeedForwardVolts = 0.0;

        org.littletonrobotics.junction.Logger.recordOutput("fix", calculate(robotHeading, targetRad, state));

        turretMotor.setControl(m_request
                .withPosition(calculate(robotHeading, targetRad, state))
                .withVelocity(currentMaxVel)
                .withAcceleration(currentMaxAccel)
                .withJerk(BASE_JERK)
                .withFeedForward(extraFeedForwardVolts));
    }

    @Override
    public Angle getAngle() {
        this.turretPosition.refresh();
        return Radians.of(turretPosition.getValue().in(Radians));
    }

    @Override
    public Angle getAnglegoal() {
        return goal;
    }

    @Override
    public boolean isAtSetPosition() {
        return Math.abs(turretMotor.getClosedLoopError().getValueAsDouble()) < (40.0 / 360.0);
    }

    @Override
    public Command sysid() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    SignalLogger.start();
                    turretMotor.getPosition().setUpdateFrequency(250);
                    turretMotor.getVelocity().setUpdateFrequency(250);
                    turretMotor.getMotorVoltage().setUpdateFrequency(250);
                }),

                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                        .until(() -> this.getAngle().in(Radians) > ShooterConstants.SOFT_MAX_LIMIT),

                new WaitCommand(1.5),

                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                        .until(() -> this.getAngle().in(Radians) < ShooterConstants.SOFT_MIN_LIMIT),

                new WaitCommand(1.5),

                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                        .until(() -> this.getAngle().in(Radians) > ShooterConstants.SOFT_MAX_LIMIT),

                new WaitCommand(1.5),

                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                        .until(() -> this.getAngle().in(Radians) < ShooterConstants.SOFT_MIN_LIMIT),

                Commands.runOnce(() -> {
                    SignalLogger.stop();
                    turretMotor.getPosition().setUpdateFrequency(50);
                    turretMotor.getVelocity().setUpdateFrequency(50);
                    turretMotor.getMotorVoltage().setUpdateFrequency(50);
                }));
    }
}