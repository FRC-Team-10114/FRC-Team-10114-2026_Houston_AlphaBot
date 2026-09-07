package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.*; // 引入所有單位

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodIOTalon implements HoodIO {

    private final TalonFX HoodMotor;
    private final CANcoder Hoodcancoder;
    private final StatusSignal<Angle> HoodPosition;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(Degree.of(0));

    private final double rotorToSensorRatio = ShooterConstants.HoodCancoder_GEAR_RATIO_TOMotor;

    private final double sensorToMechRatio = ShooterConstants.Hood_GEAR_RATIO
            / ShooterConstants.HoodCancoder_GEAR_RATIO_TOMotor;
    private final VoltageOut voltagRequire = new VoltageOut(0.0);
    private final SysIdRoutine sysIdRoutine;

    private double latestTargetDegrees = 0.0;

    public HoodIOTalon() {
        this.HoodMotor = new TalonFX(IDs.Shooter.HOOD_MOTOR, "canivore");
        this.Hoodcancoder = new CANcoder(IDs.Shooter.HOOD_CANCODER, "canivore");
        this.HoodPosition = HoodMotor.getPosition();

      SignalLogger.setPath("/U/");

        this.sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(3),
                        null, (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> this.HoodMotor.setControl(voltagRequire.withOutput(volts.in(Volts))),
                        null,
                        new SubsystemBase() {
                            @Override
                            public String getName() {
                                return "TurretSysId";
                            }
                        }));

        CANcoderConfig();

        configure();

        // initStartingPosition();
    }

    private void initStartingPosition() {

        StatusSignal<Angle> currentPos = HoodMotor.getPosition();

        currentPos.waitForUpdate(0.25);

        m_request.Position = currentPos.getValueAsDouble();
    }

    @Override
    public void CANcoderConfig() {
        var cfg = new CANcoderConfiguration();

        double targetSensorRotations = Units.degreesToRotations(20.277) * sensorToMechRatio;

        cfg.MagnetSensor.MagnetOffset = 0.3115234375 + targetSensorRotations;

        cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        Hoodcancoder.getConfigurator().apply(cfg);
    }

    public void configure() {
        var hoodConfig = new TalonFXConfiguration();

        hoodConfig.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(15);

        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        hoodConfig.SoftwareLimitSwitch
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(ShooterConstants.Hood_MIN_LIMIT)
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(ShooterConstants.Hood_MAX_LIMIT);

        hoodConfig.Feedback
                .withFeedbackRemoteSensorID(IDs.Shooter.HOOD_CANCODER)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withRotorToSensorRatio(rotorToSensorRatio)
                .withSensorToMechanismRatio(sensorToMechRatio);

        hoodConfig.Slot0.kP = 80.0;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 0.0;
        hoodConfig.Slot0.kG = 0.29103;
        hoodConfig.Slot0.kA = 0.099301;
        hoodConfig.Slot0.kS = 0.21953;
        hoodConfig.Slot0.kV = 4.5976;
        hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;


        hoodConfig.MotionMagic
                .withMotionMagicCruiseVelocity(DegreesPerSecond.of(1080))
                .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(3600));

        HoodMotor.getConfigurator().apply(hoodConfig);
    }

    @Override
    public void setAngle(Angle angle) {
        this.latestTargetDegrees = angle.in(Degrees);
        HoodMotor.setControl(m_request.withPosition(angle));
    }

    @Override
    public double getAngle() {
        this.HoodPosition.refresh();

        return this.HoodPosition.getValue().in(Degrees);
    }

    @Override
    public boolean isAtSetPosition() {
        this.HoodPosition.refresh();

        double currentDegrees = this.HoodPosition.getValue().in(Degrees);

        double error = Math.abs(this.latestTargetDegrees - currentDegrees);

        return error < 1.5;
    }
          public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.dynamic(direction);
    }

public Command startCommand() {
        System.err.println("start");
        return Commands.runOnce(SignalLogger::start);
    }

    public Command stopCommand() {
        System.err.println("end");
        return Commands.runOnce(SignalLogger::stop);
    }

public Command sysIdTest() {
        return Commands.sequence(
            this.startCommand(),
            this.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                .until(() -> this.getAngle() > 50.0),
            
            new WaitCommand(0.5),

            this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                .until(() -> this.getAngle() < 30.0),
            
            new WaitCommand(0.5),

            this.sysIdDynamic(SysIdRoutine.Direction.kForward)
                .until(() -> this.getAngle() > 43.0),
            
            new WaitCommand(0.5),

            this.sysIdDynamic(SysIdRoutine.Direction.kReverse)
                .until(() -> this.getAngle() < 30.0),
            this.stopCommand()
        );
    }
}