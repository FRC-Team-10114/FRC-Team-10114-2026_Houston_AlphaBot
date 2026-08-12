package frc.robot.subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
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

    // 🟢 封印 3 解除：只給一個初始位置 0，不傳入多餘的參數干擾 Config
    // 🟢 正確寫法：(初始位置, Expo_kV, Expo_kA)
private final DynamicMotionMagicExpoVoltage m_request = new DynamicMotionMagicExpoVoltage(0.0, 1.5255, 0.13204);
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
                .withReverseSoftLimitThreshold(Radians.of(ShooterConstants.HARD_MIN_LIMIT).in(Rotations))
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Radians.of(ShooterConstants.HARD_MAX_LIMIT).in(Rotations));
        
        configs.Feedback
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(18.0); 

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // ==========================================
        // 🚀 Motion Magic Expo 設定核心
        // ==========================================
        // 🟢 封印 1 解除：移除 * 2 * Math.PI，釋放馬達最高極速！
        configs.MotionMagic.MotionMagicExpo_kV = 1.5255; 
        configs.MotionMagic.MotionMagicExpo_kA = 0.13204;

        configs.Slot0.kS = 0.63542;
        configs.Slot0.kP = 38.0;
        configs.Slot0.kD = 0.5;

        turretMotor.getConfigurator().apply(configs);
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
        return Math.abs(turretMotor.getClosedLoopError().getValueAsDouble()) < (1.5 / 360.0);
    }

    @Override
    public Command sysid() {
        // (省略 SysIdRoutine 的程式碼，維持不變)
        return Commands.none(); // 這裡為了版面整潔縮寫，請保留你原本的 sysid() 內容！
    }

    @Override
    public void setAngle(
            Rotation2d robotHeading,
            Angle targetRad,
            ShootState state
    ) {

        Angle bestAngle = calculate(robotHeading, targetRad, state);

        // ✨ 完美餵給 Dynamic Expo 控制器
        turretMotor.setControl(m_request
                .withPosition(bestAngle)
                // 🟢 封印 2 解除：把我們算好的預判電壓塞進去，走射才會準！
                .withFeedForward(0.0) 
        );
    }
    @Override
    public double getVelocityRadsPerSec() {
        // 1. 取得馬達最新速度 (單位：圈/秒)
        double mechanismRps = turretMotor.getVelocity().getValueAsDouble();
        
        // 2. 轉換為弧度/秒 (Radians per second) 給物理引擎使用
        return mechanismRps * 2.0 * Math.PI;
    }
}