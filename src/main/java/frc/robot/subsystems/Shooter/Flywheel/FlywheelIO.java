package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.units.measure.AngularVelocity;

public interface FlywheelIO {
    
    public void setRPS(AngularVelocity RPS);

    public AngularVelocity getRPS();

    public boolean isAtSetPosition();
}
