package frc.robot.subsystems.Intake.Arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public interface ArmIO {

    public void setPosition(Angle angle);

    public double getPosition();

    public void resetEncoder();

    public void configure();

    public Command sysid();
}
