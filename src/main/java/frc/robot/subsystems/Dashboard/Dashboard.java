package frc.robot.subsystems.Dashboard;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FMS.Signal;

public class Dashboard extends SubsystemBase {

    private double matchTime, roundTime;
    private final Signal fms;

    private boolean endgameAlertTriggered = false;

    private boolean timeAlert = false;
    
    private String status = "Waiting";

    private final String[] statusArray = {"Round 1", "Round 2", "Round 3", "Round 4"};

    private final double MatchTime = 140.0;
    private final double TRANSITION = 10.0;
    private final double Round = 25.0;

    public Dashboard(Signal fms) {
        this.fms = fms;
        this.matchTime = -1;
        this.roundTime = -1;
    }

    @Override
    public void periodic() {
        updateMatchData();
        try {
            updateRoundTime();
        } catch (Exception e) {
            System.err.println(e);
        }
        logDashboardData();
    }

    private void updateMatchData() {
        matchTime = DriverStation.getMatchTime();
    }

    private void logDashboardData() {
        Logger.recordOutput("Dashboard/MatchTime", matchTime);
        Logger.recordOutput("Dashboard/RoundTime", roundTime);
        Logger.recordOutput("Dashboard/isActive", fms.Active());
        Logger.recordOutput("Dashboard/TimeAlert", timeAlert);
        Logger.recordOutput("Dashboard/Status", status);

        Logger.recordOutput("Dashboard/BatteryVoltage", RobotController.getBatteryVoltage());

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            Logger.recordOutput("Dashboard/Alliance", alliance.get().toString());
        } else {
            Logger.recordOutput("Dashboard/Alliance", "Unknown");
        }

        timeAlert();
    }

    private void updateRoundTime() {
        if (DriverStation.isAutonomous()) {
            roundTime = DriverStation.getMatchTime();
            status = "Autonomous";
            return;
        }
        if (DriverStation.getMatchTime() >= MatchTime - TRANSITION) {
            roundTime = DriverStation.getMatchTime() - (MatchTime - TRANSITION);
            status = "Transition";
            return;
        }
        for (int round = 1; round <= 4; round++) {
            if (fms.isInRound(round)) {
                roundTime = DriverStation.getMatchTime() - 30 - ((4-round) * Round);
                status = statusArray[round - 1];
                return;
            }
        }

        roundTime = this.matchTime;
        status = "End Game";
        return;
    }

    public void timeAlert() {
        if (roundTime <= 10) {
            alertBlink();
        } else {
            timeAlert = false;
        }
    }

    private void alertBlink() {
        timeAlert = (int) (Timer.getFPGATimestamp() * 2) % 2 == 0;
    }
}