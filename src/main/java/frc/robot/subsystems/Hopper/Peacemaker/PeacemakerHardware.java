package frc.robot.subsystems.Hopper.Peacemaker;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IDs;

public class PeacemakerHardware implements PeacemakerIO{
    private final TalonFX Motor;

    private final VoltageOut output;

    public PeacemakerHardware() {
        this.Motor = new TalonFX(48, "canivore");
        this.output = new VoltageOut(Volts.of(0)); 

        configure();
    }

    @Override
    public void setVoltage(Voltage volt) {
        this.Motor.setControl(output.withOutput(volt));
    }

    @Override
    public void stop() {
        this.Motor.stopMotor();
    }

    @Override
    public void configure() {
        var mechineConfig = new TalonFXConfiguration();

        mechineConfig.MotorOutput
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
        mechineConfig.CurrentLimits
                .withStatorCurrentLimit(Amp.of(40))
                .withSupplyCurrentLimit(Amp.of(30))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        
        Motor.getConfigurator().apply(mechineConfig);
    }
}
