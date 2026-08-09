package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDs.LED;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.RobotEvent.Event.ShootingStateFalse;
import frc.robot.util.RobotEvent.Event.ShootingStateTrue;

public class superstructure extends SubsystemBase {

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final List<ShootingStateTrue> ShootingStateTrue = new ArrayList<>();
    private final List<ShootingStateFalse> ShootingStateFalse = new ArrayList<>();

    public superstructure(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            HopperSubsystem hopper) {
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
    }

    public Command intake() {
        return this.intake.intake();
    }

    public Command stopintake() {
        return Commands.parallel(
                Commands.runOnce(intake::rollerEnd));
    }

    public Command armforshoot() {
        return this.intake.shootintake();
    }

    public void shoot() {
        this.intake.rollerStart();
    }

    public Command shootCommand() {
        return Commands.parallel(

                // 2. 執行射擊與供彈判斷的 Command
                Commands.run(() -> {
                    this.setShootingStateTrue();

                    this.shooter.shoot();

                    if (shooter.isAtSetPosition()) {
                        this.hopper.warmUpforshoot();
                    } else {
                        this.hopper.warmUpforshoot();
                    }
                }, this.shooter, this.hopper));
    }

    public Command stopShoot() {
        return Commands.parallel(
                Commands.runOnce(this::setShootingStateFalse),
                Commands.runOnce(this.shooter::stopShoot),
                Commands.runOnce(hopper::stop));
    }

    public void setShootingStateTrue() {
        for (ShootingStateTrue listener : ShootingStateTrue) {
            listener.ShootingStateTrue();
        }
    }

    public void setShootingStateFalse() {
        for (ShootingStateFalse listener : ShootingStateFalse) {
            listener.ShootingStateFalse();
        }
    }
    public Command autoshoot() {
        return Commands.sequence(shootCommand().withTimeout(4.0));
    }
    
    public Command autointake() {
        return Commands.parallel(this.intake());
    }

    public Command intakestop() {
        return Commands.parallel(this.stopintake());
    }
    public Command keepsafe(){
        return this.shooter.waithoodsafe();
    }
}
