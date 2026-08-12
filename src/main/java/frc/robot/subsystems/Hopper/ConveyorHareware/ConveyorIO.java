package frc.robot.subsystems.Hopper.ConveyorHareware;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ConveyorIO {
    
    public void setRPS(AngularVelocity rps);

    public AngularVelocity getRPS();

    public void stop();

    public boolean isAtSetPosition();

}