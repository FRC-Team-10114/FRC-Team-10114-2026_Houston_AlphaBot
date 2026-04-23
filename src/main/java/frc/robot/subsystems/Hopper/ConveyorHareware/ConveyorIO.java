package frc.robot.subsystems.Hopper.ConveyorHareware;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ConveyorIO {
    public void setVoltage(Voltage volt);

    public void stop();

    public void configure();
}
