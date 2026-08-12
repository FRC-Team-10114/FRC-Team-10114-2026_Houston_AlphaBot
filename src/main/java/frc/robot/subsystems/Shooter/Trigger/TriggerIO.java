package frc.robot.subsystems.Shooter.Trigger;

import edu.wpi.first.units.measure.AngularVelocity;

public interface TriggerIO {
    
    public void setRPS(AngularVelocity rps);

    public AngularVelocity getRPS();

    public void stop();

    public boolean isAtSetPosition();

}