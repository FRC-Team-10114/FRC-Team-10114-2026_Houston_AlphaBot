package frc.robot.subsystems.Intake.Roller;

import edu.wpi.first.units.measure.AngularVelocity;

public interface RollerIO {
    
    public void setRPS(AngularVelocity rps);

    public AngularVelocity getRPS();

    public void stop();
}