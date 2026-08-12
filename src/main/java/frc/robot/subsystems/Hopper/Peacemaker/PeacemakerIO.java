package frc.robot.subsystems.Hopper.Peacemaker;

import edu.wpi.first.units.measure.AngularVelocity;

public interface PeacemakerIO {
    
    public void setRPS(AngularVelocity rps);

    public AngularVelocity getRPS();

    public void stop();

    public boolean isAtSetPosition();

}