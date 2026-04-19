package frc.robot.subsystems.Intake.Arm;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeConstants.ArmConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class ArmIOTalon implements ArmIO {

        private final TalonFX armMotor;

        private final TalonFX followmotor;

        private final StatusSignal<Angle> armPosition;

        private final MotionMagicVoltage armOutput = new MotionMagicVoltage(Degree.of(0));

        private final SysIdRoutine sysIdRoutine;
        private final VoltageOut voltagRequire = new VoltageOut(0.0);

        public ArmIOTalon() {

                this.armMotor = new TalonFX(IDs.Intake.ARM_MOTOR, "canivore");
                this.followmotor = new TalonFX(IDs.Intake.Arm_follow, "canivore");
                this.armPosition = armMotor.getPosition();
                followmotor.setControl(new Follower(armMotor.getDeviceID(), MotorAlignmentValue.Opposed));
                configure();
                resetEncoder();

                SignalLogger.setPath("/U/");

                this.sysIdRoutine = new SysIdRoutine(
                                new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(1),
                                                null, (state) -> SignalLogger.writeString("state", state.toString())),
                                new SysIdRoutine.Mechanism(
                                                (volts) -> this.armMotor
                                                                .setControl(voltagRequire.withOutput(volts.in(Volts))),
                                                null,
                                                new SubsystemBase() {
                                                        @Override
                                                        public String getName() {
                                                                return "TurretSysId";
                                                        }
                                                }));
        }

        @Override
        public void setPosition(Angle angle) {
                armMotor.setControl(armOutput.withPosition(angle));
        }

        @Override
        public double getPosition() {
                this.armPosition.refresh();
                return this.armPosition.getValue().in(Degrees);
        }

        @Override
        public void resetEncoder() {
                this.armMotor.getConfigurator().setPosition(Degree.of(137.373046875));
        }

        @Override
        public void configure() {
                var IntakeArmConfig = new TalonFXConfiguration();

                IntakeArmConfig.CurrentLimits
                                .withStatorCurrentLimitEnable(true)
                                .withStatorCurrentLimit(40)
                                .withSupplyCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(20);

                IntakeArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                IntakeArmConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                IntakeArmConfig.SoftwareLimitSwitch
                                .withReverseSoftLimitEnable(true)
                                .withReverseSoftLimitThreshold(Degree.of(-5))
                                .withForwardSoftLimitEnable(true)
                                .withForwardSoftLimitThreshold(Degree.of(135));

                IntakeArmConfig.Feedback.SensorToMechanismRatio = 10.0;

                IntakeArmConfig.Slot0.kP = 120.0;
                IntakeArmConfig.Slot0.kI = 0.0;
                IntakeArmConfig.Slot0.kD = 3.5;
                IntakeArmConfig.Slot0.kG = 0.0;
                IntakeArmConfig.Slot0.kA = 0.62178;
                IntakeArmConfig.Slot0.kS = 0.41393;
                IntakeArmConfig.Slot0.kV = 0.14832;
                IntakeArmConfig.Slot0.kG = 0.36386;
                IntakeArmConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

                IntakeArmConfig.MotionMagic
                                .withMotionMagicCruiseVelocity(DegreesPerSecond.of(1080))
                                .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(3600));

                armMotor.getConfigurator().apply(IntakeArmConfig);
        }

        @Override
        public Command sysid() {
                return Commands.sequence(
                                Commands.runOnce(() -> {
                                        SignalLogger.start();
                                        armMotor.getPosition().setUpdateFrequency(250);
                                        armMotor.getVelocity().setUpdateFrequency(250);
                                        armMotor.getMotorVoltage().setUpdateFrequency(250);
                                }),

                                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                                                .until(() -> this.getPosition() < 55),
                                new WaitCommand(1.5),

                                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                                                .until(() -> this.getPosition() > 135),

                                new WaitCommand(1.5),

                                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                                                .until(() -> this.getPosition() < 55),

                                new WaitCommand(1.5),

                                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                                                .until(() -> this.getPosition() > 135),

                                Commands.runOnce(() -> {
                                        SignalLogger.stop();
                                        armMotor.getPosition().setUpdateFrequency(50);
                                        armMotor.getVelocity().setUpdateFrequency(50);
                                        armMotor.getMotorVoltage().setUpdateFrequency(50);
                                }));
        }

}
