package frc.robot.subsystems.Shooter.Hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public interface HoodIO {

    public void setAngle(Angle angle);

    public double getAngle();

    public void CANcoderConfig();

    public void configure();

    public boolean isAtSetPosition();

    public Command sysIdTest();

}
