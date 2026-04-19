package frc.robot.util.FMS;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotEvent.Event.*;

public class Signal extends SubsystemBase { // 類別名稱習慣大寫開頭
    public String gameData;
    public boolean CanGetPoint;
    private final List<TargetInactive> TargetInactive = new ArrayList<>();
    private final List<Targetactive> Targetactive = new ArrayList<>();
    private final double MatchTime = 140.0;
    private final double TRANSITION = 10.0;
    private final double Round = 25.0;

    public Signal() {
    }

    public char getAllianceChar() {
        char alliance;

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            alliance = 'R';
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
            alliance = 'B';
        } else {
            alliance = 'N';
        }

        return alliance;
    }

    public char getInactive() {
        gameData = DriverStation.getGameSpecificMessage();

        char allience = 'N';

        if (gameData != null && gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    allience = 'B';
                    break;
                case 'R':
                    allience = 'R';
                    break;
                default:
                    allience = 'N';
                    break;
            }
        }
        return allience;
    }

    public boolean isInactive() {
        return getAllianceChar() == getInactive();
    }

    public void TargetInactive(TargetInactive event) {
        TargetInactive.add(event);
    }

    public void Targetactive(Targetactive event) {
        Targetactive.add(event);
    }

    public void publishActive() {
        for (Targetactive listener : Targetactive) {
            listener.Targetactive();
        }
    }

    public void publishInactive() {
        for (TargetInactive listener : TargetInactive) {
            listener.TargetInactive();
        }
    }

    public void CanGetPointAllience() {
        if (DriverStation.isAutonomous()) {
            publishActive();
        }
        if (isTRANSITION()) {
            publishActive();
        } else if (isInRound(1)) {
            if (!isInactive())
                publishActive();
            else
                publishInactive();
        } else if (isInRound(2)) {
            if (isInactive())
                publishActive();
            else
                publishInactive();
        } else if (isInRound(3)) {
            if (!isInactive())
                publishActive();
            else
                publishInactive();
        } else if (isInRound(4)) {
            if (isInactive())
                publishActive();
            else
                publishInactive();
        } else {
            publishActive();
        }

    }

    public boolean Active() {
        if (DriverStation.isAutonomous()) {
            return true;
        }
        if (isTRANSITION()) {
            return true;
        } else if (isInRound(1)) {
            if (!isInactive())
                return true;
            else
                return false;
        } else if (isInRound(2)) {
            if (isInactive())
                return true;
            else
                return false;
        } else if (isInRound(3)) {
            if (!isInactive())
                return true;
            else
                return false;
        } else if (isInRound(4)) {
            if (isInactive())
                return true;
            else
                return false;
        } else {
            return true;
        }
    }

    public boolean isInRound(int Round) {
        if (DriverStation.getMatchTime() <= MatchTime - TRANSITION - (Round - 1 * this.Round) &&
                DriverStation.getMatchTime() >= MatchTime - TRANSITION - (Round * this.Round)) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isTRANSITION() {
        if (DriverStation.getMatchTime() >= MatchTime - TRANSITION) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        CanGetPointAllience();
    }
}