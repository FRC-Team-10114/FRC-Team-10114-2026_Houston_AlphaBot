package frc.robot.subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    private final StatusSignal<Angle> turretPosition;

    private final double gearRatio = (96.0 / 16.0) * 3.0;

    // 🟢 替換為 Expo 專屬的 Request (不再需要設定 V, A, J 極限)
    private final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
    private final VoltageOut voltagRequire = new VoltageOut(0.0);
    public Angle goal;

    private final SysIdRoutine sysIdRoutine;

    public TurretHardware() {
        this.turretMotor = new TalonFX(IDs.Shooter.TURRET_MOTOR, "canivore");
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

        configureMotors();

        // 🟢 開局強制將馬達當前位置歸零 (因為沒有 CANcoder 了)
        // ⚠️ 警告：機器人開機時，砲塔必須停在正中間 (0度) 的位置！
        this.turretMotor.setPosition(0.0);
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
                // 🟢 明確宣告這是弧度，並轉換成圈數 (Rotations) 給馬達
                .withReverseSoftLimitThreshold(Radians.of(ShooterConstants.HARD_MIN_LIMIT).in(Rotations))
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Radians.of(ShooterConstants.HARD_MAX_LIMIT).in(Rotations));
        // 🟢 恢復使用馬達內部感測器 (RotorSensor)
        configs.Feedback
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(18.0); // 傳動比

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // ==========================================
        // 🚀 Motion Magic Expo 設定核心
        // ==========================================
        // Expo 需要靠 kV 和 kA 來規劃完美曲線，所以要寫在 MotionMagic 區塊裡
        configs.MotionMagic.MotionMagicExpo_kV = 1.5255 * 2 * Math.PI;
        configs.MotionMagic.MotionMagicExpo_kA = 0.13204 * 2 * Math.PI;

        // Slot0 只需要負責靜摩擦力 (kS) 與誤差修正 (kP, kD)
        configs.Slot0.kS = 0.63542;
        configs.Slot0.kP = 12.0;
        configs.Slot0.kD = 0.5;
        // 注意：這裡不需要再設定 Slot0.kV 和 Slot0.kA，Expo 會自己處理！

        turretMotor.getConfigurator().apply(configs);
    }

    @Override
    public void setAngle(Rotation2d robotHeading, Angle targetRad, ShootState state) {
        // ✨ 修正 1：不要轉成 double，直接保留 Angle 物件
        Angle bestAngle = calculate(robotHeading, targetRad, state);

        double extraFeedForwardVolts = 0.0;

        Logger.recordOutput("fix", bestAngle.in(Radians)); // 僅 Log 使用弧度

        // ✨ 修正 2：直接傳入 Angle，Phoenix 6 會自動轉換為 Rotations
        turretMotor.setControl(m_request
                .withPosition(bestAngle)
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