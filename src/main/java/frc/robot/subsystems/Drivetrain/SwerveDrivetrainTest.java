package frc.robot.subsystems.Drivetrain;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveDrivetrainTest extends SubsystemBase {

    private final StatusSignal<Current> driveStator, driveSupply, driveTorque, steerStator, steerSupply, steerTorque;
    private final StatusSignal<Voltage> driveVoltage, steerVoltage;
    private final int ModuleIndex;
    
    private int loopCounter = 0;
    private final int LOG_INTERVAL = 10;

    public SwerveDrivetrainTest(CommandSwerveDrivetrain drivetrain, int moduleIndex) {
        this.ModuleIndex = moduleIndex;
        TalonFX driveMotor = drivetrain.getModule(moduleIndex).getDriveMotor();
        TalonFX steerMotor = drivetrain.getModule(moduleIndex).getSteerMotor();
        
        this.driveStator = driveMotor.getStatorCurrent();
        this.driveSupply = driveMotor.getSupplyCurrent();
        this.driveTorque = driveMotor.getTorqueCurrent();
        this.driveVoltage = driveMotor.getMotorVoltage();

        this.steerStator = steerMotor.getStatorCurrent();
        this.steerSupply = steerMotor.getSupplyCurrent();
        this.steerTorque = steerMotor.getTorqueCurrent();
        this.steerVoltage = steerMotor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(10.0, 
            driveStator, driveSupply, driveTorque, driveVoltage,
            steerStator, steerSupply, steerTorque, steerVoltage
        );
    }

    @Override
    public void periodic() {
        loopCounter++;
        if (loopCounter >= LOG_INTERVAL) {
            refreshSignal();
            logMotorCurrents();
            loopCounter = 0;
        }
    }

    public void refreshSignal() {
        BaseStatusSignal.refreshAll(
            driveStator, driveSupply, driveTorque, driveVoltage,
            steerStator, steerSupply, steerTorque, steerVoltage
        );
    }

    public void logMotorCurrents() {
        String path = "Drivetrain/CurrentTest/" + SwerveModuleConstants.ModuleName[ModuleIndex];

        Logger.recordOutput(path + "/DriveMotor/StatorCurrent", driveStator.getValueAsDouble());
        Logger.recordOutput(path + "/DriveMotor/SupplyCurrent", driveSupply.getValueAsDouble());
        Logger.recordOutput(path + "/DriveMotor/TorqueCurrent", driveTorque.getValueAsDouble());
        Logger.recordOutput(path + "/DriveMotor/Voltage", driveVoltage.getValueAsDouble()); 
        
        boolean driveStall = driveStator.getValueAsDouble() > 60.0;
        Logger.recordOutput("Drivetrain/Warnings/" + SwerveModuleConstants.ModuleName[ModuleIndex] + "/DriveMotorHighCurrentStall", driveStall);
    }
}