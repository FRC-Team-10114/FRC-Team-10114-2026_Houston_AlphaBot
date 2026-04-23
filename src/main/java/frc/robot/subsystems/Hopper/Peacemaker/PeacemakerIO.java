package frc.robot.subsystems.Hopper.Peacemaker;

import edu.wpi.first.units.measure.Voltage;

public interface PeacemakerIO {
    public void setVoltage(Voltage volt);

    public void stop();

    public void configure();
}
