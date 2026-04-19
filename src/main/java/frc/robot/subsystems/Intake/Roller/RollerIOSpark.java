package frc.robot.subsystems.Intake.Roller;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Intake.IntakeConstants.RollerConstants;

public class RollerIOSpark implements RollerIO {

    private final SparkFlex rollerMotor;
    private final RelativeEncoder rollerEncoder;
    private final SparkClosedLoopController rollerController;

    public RollerIOSpark() {
        this.rollerMotor = new SparkFlex(IDs.Intake.ROLLER_MOTOR, MotorType.kBrushless);
        this.rollerEncoder = rollerMotor.getEncoder();
        this.rollerController = rollerMotor.getClosedLoopController();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        this.rollerController.setSetpoint(voltage.baseUnitMagnitude(), ControlType.kVoltage);
    }

    @Override
    public AngularVelocity getVelocity() {
        return RadiansPerSecond.of(this.rollerEncoder.getPosition());
    }

    @Override
    public void configure() {
        var rollerConfig = new SparkFlexConfig();

        rollerConfig
        
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit((int) RollerConstants.SUPPLY_CURRENT_LIMIT.baseUnitMagnitude())
                .apply(rollerConfig);
        rollerConfig.closedLoop
                .pid(
                        RollerConstants.PID[0], 
                        RollerConstants.PID[1], 
                        RollerConstants.PID[2])
                .apply(rollerConfig.closedLoop);
        rollerConfig.encoder
                .velocityConversionFactor(RollerConstants.VELOCITY_CONVERSION_FACOTR)
                .positionConversionFactor(RollerConstants.POSITION_CONVERSION_FACTOR)
                .apply(rollerConfig.encoder);

        rollerMotor.configure(
                rollerConfig, 
                ResetMode.kResetSafeParameters, 
                PersistMode.kPersistParameters);
    }
    
}
