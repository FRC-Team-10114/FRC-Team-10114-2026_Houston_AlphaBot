package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

public class Telemetry {

    public Telemetry(SwerveDriveState swerveDriveState) {}

    public void telemeterize(SwerveDriveState state) {
        // Chassis pose and speeds
        Logger.recordOutput("DriveState/Pose", state.Pose);
        Logger.recordOutput("DriveState/Speeds", state.Speeds);
        // Module states, targets, and positions
        Logger.recordOutput("DriveState/ModuleStates", state.ModuleStates);
        Logger.recordOutput("DriveState/ModuleTargets", state.ModuleTargets);
        Logger.recordOutput("DriveState/ModulePositions", state.ModulePositions);
        // Additional drivetrain data
        Logger.recordOutput("DriveState/OdometryPeriod", state.OdometryPeriod);
        Logger.recordOutput("DriveState/Timestamp", state.Timestamp);
    }
}