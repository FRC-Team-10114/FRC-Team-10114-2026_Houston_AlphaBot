package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Arm.ArmIO;
import frc.robot.subsystems.Intake.Arm.ArmIOTalon;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIOSpark;

public class IntakeSubsystem extends SubsystemBase {

    private final ArmIO arm;

    private final RollerIO roller;

    private intakestate state = intakestate.none;

    public IntakeSubsystem(ArmIO arm, RollerIO roller) {
        this.arm = arm;
        this.roller = roller;
    }

    public static IntakeSubsystem create() {
        return new IntakeSubsystem(
                new ArmIOTalon(),
                new RollerIOSpark());
    }

    public enum intakestate {
        suck, none;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("intakearmangle", this.arm.getPosition());
    }

    public void rollerStart() {
        this.roller.setVoltage(Volts.of(4.0));
    }

    public void rollerEnd() {
        this.roller.setVoltage(Volts.of(0));
    }

    public void armup() {
        state = intakestate.none;
        this.arm.setPosition(Degrees.of(82));
    }

    
    public void armdownforshoot() {
        this.arm.setPosition(Degrees.of(82));
    }

    public void armupforshoot() {     
        this.arm.setPosition(Degrees.of(72));
    }

    public void armdown() {
        state = intakestate.suck;
        this.arm.setPosition(Degrees.of(-2.5));
    }
    public void armupforclimb(){
        this.arm.setPosition(Degrees.of(135));
    }

    public Command intake() {
        return Commands.sequence(
                Commands.runOnce(this::armdown, this),
                Commands.runOnce(this::rollerStart, this));
    }

    public Command shootintake() {
        return Commands.either(Commands.repeatingSequence(
                Commands.runOnce(this::armdownforshoot, this),

                Commands.waitSeconds(0.2),

                Commands.runOnce(this::armupforshoot, this),

                Commands.waitSeconds(0.2)), Commands.none(), () -> state == intakestate.none);

    }
    public Command sysid(){
        return this.arm.sysid();
    }
    public void roll(){
        this.roller.setVoltage(Volts.of(5));
    }
}
