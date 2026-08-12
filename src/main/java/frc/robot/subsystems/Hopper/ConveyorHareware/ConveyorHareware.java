package frc.robot.subsystems.Hopper.ConveyorHareware;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
// 修正 1: 改用速度控制專用的 Request
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.subsystems.Shooter.ShooterConstants;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class ConveyorHareware implements ConveyorIO {

    private final TalonFX roller = new TalonFX(30, "canivore");

    private final StatusSignal<AngularVelocity> velocitySignal = roller.getVelocity();

    private final VelocityTorqueCurrentFOC m_request = new VelocityTorqueCurrentFOC(0);

    private double targetRPS = 0.0;

    private final StatusSignal<Current> statorCurrentSignal;

    public ConveyorHareware() {
        velocitySignal.setUpdateFrequency(50);

        this.statorCurrentSignal = roller.getStatorCurrent();
        statorCurrentSignal.setUpdateFrequency(50);

        this.configureMotors();
    }

    public void configureMotors() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(40.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20.0); // 60 -> 40

        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        configs.Slot0.kP = 3.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;

        configs.Slot0.kV = 0.0;

        configs.Slot0.kS = 0.0;

        configs.Feedback.SensorToMechanismRatio = 1.0 / 3.0;

        roller.getConfigurator().apply(configs);
    }

    @Override
    public void setRPS(AngularVelocity RPS) {
        double targetRPS = RPS.in(RotationsPerSecond);

        this.targetRPS = targetRPS;

        roller.setControl(m_request.withVelocity(targetRPS));
    }

    @Override
    public AngularVelocity getRPS() {
        velocitySignal.refresh();

        return velocitySignal.getValue();
    }

    @Override
    public boolean isAtSetPosition() {
        // 1. 刷新數據
        velocitySignal.refresh();

        // 2. 取得目前實際轉速
        double currentRPS = velocitySignal.getValue().in(RotationsPerSecond);

        // 3. 計算誤差絕對值
        double error = Math.abs(targetRPS - currentRPS);

        if (Math.abs(targetRPS) < 0.1) {
            return Math.abs(currentRPS) < 1.0;
        }

        return error <= 3;
    }

    @Override
    public void stop() {
        this.roller.stopMotor();
    }

}