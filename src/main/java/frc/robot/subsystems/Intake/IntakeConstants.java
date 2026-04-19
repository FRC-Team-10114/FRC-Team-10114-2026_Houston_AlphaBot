package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class IntakeConstants {

    public static final class ArmConstants {
        public static final double GEAR_RATIO = 22.5;
        public static final double POSITION_CONVERSION_FACTOR = Degree.convertFrom(GEAR_RATIO,
                Rotation);

        public static final double[] PID = { 0.04, 0.0, 0.001};
        public static final double KG = 0;  // TODO 用tuner測
        public static final double CRUISE_VELOCITY = 0.25;
        public static final double MAX_ACCELERATION = 0.5;

        public static final Current STATOR_CURRENT_LIMIT = Amp.of(40);
        public static final Current SUPPLY_CURRENT_LIMIT = Amp.of(15);

        public static final Angle FORWARD_LIMIT = Rotation.of(0.3);
        public static final Angle REVERSE_LIMIT = Rotation.of(0);

    }

    public static final class RollerConstants {
        public static final double GEAR_RATIO = 1;
        public static final double VELOCITY_CONVERSION_FACOTR = RotationsPerSecond
                .convertFrom(GEAR_RATIO, RotationsPerSecond);
        public static final double POSITION_CONVERSION_FACTOR = Rotation.convertFrom(
                GEAR_RATIO, Rotation);
        public static final double[] PID = { 0.1, 0.0, 0.0 };

        public static final Current STATOR_CURRENT_LIMIT = Amp.of(40);

        public static final Current SUPPLY_CURRENT_LIMIT = Amp.of(45);
    }
}