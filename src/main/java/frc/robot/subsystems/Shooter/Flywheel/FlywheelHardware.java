package frc.robot.subsystems.Shooter.Flywheel;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Shooter.ShooterConstants;

import static edu.wpi.first.units.Units.*;

public class FlywheelHardware implements FlywheelIO {

    private final TalonFX flywheel = new TalonFX(IDs.Shooter.FLYWHEEL_MOTOR, "canivore");

    private final StatusSignal<AngularVelocity> velocitySignal = flywheel.getVelocity();

    private final VelocityTorqueCurrentFOC m_request = new VelocityTorqueCurrentFOC(0);

    private double targetRPS = 0.0;

    public FlywheelHardware() {
        velocitySignal.setUpdateFrequency(50);

        this.configureMotors();
    }

    public void configureMotors() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(80.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40);

        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


        configs.Slot0.kP = 10.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;

        configs.Slot0.kV = 0.0;

        configs.Slot0.kS = 0.0;

        configs.Feedback.SensorToMechanismRatio = ShooterConstants.Flywheel_GEAR_RATIO;

        flywheel.getConfigurator().apply(configs);
    }

    @Override
    public void setRPS(AngularVelocity RPS) {
        double targetRPS = RPS.in(RotationsPerSecond);

        this.targetRPS = targetRPS;

        flywheel.setControl(m_request.withVelocity(targetRPS));
    }

    @Override
    public AngularVelocity getRPS() {
        velocitySignal.refresh();

        return velocitySignal.getValue();
    }
    @Override
    public boolean isAtSetPosition() {
        velocitySignal.refresh();
        
        double currentRPS = velocitySignal.getValue().in(RotationsPerSecond);

        double error = Math.abs(targetRPS - currentRPS);

        if (Math.abs(targetRPS) < 0.1) {
             return Math.abs(currentRPS) < 1.0;
        }

        return error <= 3;
    }
}