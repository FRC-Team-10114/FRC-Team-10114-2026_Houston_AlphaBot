package frc.robot.subsystems.Shooter.Trigger;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IDs;
import frc.robot.subsystems.Shooter.ShooterConstants.TriggerConstants;


public class TriggerIOTalon implements TriggerIO {
    
    private final TalonFX triggerMotor;

    private final VoltageOut output;

    public TriggerIOTalon() {
        this.triggerMotor = new TalonFX(IDs.Hopper.TRIGGER_MOTOR, "canivore");
        this.output = new VoltageOut(0.0);

        configure();
    }

    @Override
    public void run() {
        this.triggerMotor.setControl(this.output.withOutput(Volt.of(11)).withEnableFOC(false));
    }

    @Override
    public void stop() {
        this.triggerMotor.stopMotor();
    }

    @Override
    public void configure() {
        var triggerConfig = new TalonFXConfiguration();

        triggerConfig.MotorOutput
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        triggerConfig.CurrentLimits
                .withStatorCurrentLimit(TriggerConstants.TRIGGER_STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimit(TriggerConstants.TRIGGER_SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        
        triggerMotor.getConfigurator().apply(triggerConfig);
    }
}
