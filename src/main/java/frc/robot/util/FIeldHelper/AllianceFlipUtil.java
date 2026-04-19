/*
 * Original code from Littleton Robotics (Team 6328) - 2026 Season
 * Modified by Team [10114]
 * * Licensed under the MIT License.
 */

package frc.robot.util.FIeldHelper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;

public class AllianceFlipUtil {
    
    public static Translation2d apply(Translation2d translation) {
        if (shouldFlip()) {
            return new Translation2d(
                FieldConstants.fieldLength - translation.getX(), 
                translation.getY()
            );
        }
        return translation;
    }

    public static Pose2d apply(Pose2d pose) {
        if (shouldFlip()) {
            return new Pose2d(
                FieldConstants.fieldLength - pose.getX(),
                pose.getY(),
                new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin())
            );
        }
        return pose;
    }

    public static boolean shouldFlip() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Red;
        }
        return false;
    }
        public static Pose2d Needapply(Pose2d pose) {
            return new Pose2d(
                FieldConstants.fieldLength - pose.getX(),
                pose.getY(),
                new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin())
            );
    }
}