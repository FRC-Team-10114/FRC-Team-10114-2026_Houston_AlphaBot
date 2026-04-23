package frc.robot.subsystems.Hopper;

import java.security.PublicKey;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.Constants.IDs.Hopper;
import frc.robot.subsystems.Hopper.ConveyorHareware.ConveyorHareware;
import frc.robot.subsystems.Hopper.ConveyorHareware.ConveyorIO;
import frc.robot.subsystems.Hopper.Peacemaker.PeacemakerHardware;
import frc.robot.subsystems.Hopper.Peacemaker.PeacemakerIO;

public class HopperSubsystem extends SubsystemBase{
    public final ConveyorIO conveyorIO;
    public final PeacemakerIO peacemakerIO;
    public HopperSubsystem(ConveyorIO conveyorIO,PeacemakerIO peacemakerIO){
        this.conveyorIO = conveyorIO;
        this.peacemakerIO = peacemakerIO;
    }
    
    public static HopperSubsystem create() {
        return new HopperSubsystem(new ConveyorHareware(),new PeacemakerHardware());
    }
    public void warmUpforshoot(){
        this.conveyorIO.setVoltage(Volts.of(9.5));
        this.peacemakerIO.setVoltage(Volts.of(3));
    }

    public void stop(){
        this.conveyorIO.stop();
        this.peacemakerIO.stop();
    }
}
