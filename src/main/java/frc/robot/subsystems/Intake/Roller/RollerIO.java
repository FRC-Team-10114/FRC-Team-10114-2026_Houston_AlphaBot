package frc.robot.subsystems.Intake.Roller;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface RollerIO {

    public void setVoltage(Voltage volt);

    public AngularVelocity getVelocity();

    public void configure();
}
