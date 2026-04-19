package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class ShooterConstants {
        public static Transform3d robotToTurret = new Transform3d(0.10167023632566, -0.00000037052800, 0.26373899694824,
                        Rotation3d.kZero);//-0.00000037052800

        public static final double HARD_MIN_LIMIT = Units.degreesToRadians(-250.0);
        public static final double HARD_MAX_LIMIT = Units.degreesToRadians(250.0);

        public static final double SOFT_MIN_LIMIT = Units.degreesToRadians(-220.0);
        public static final double SOFT_MAX_LIMIT = Units.degreesToRadians(220.0);

        public static final Angle Hood_MAX_LIMIT = Degree.of(55); // 上限63
        public static final Angle Hood_MIN_LIMIT = Degree.of(27); // 下限25

        public static final double Hood_GEAR_RATIO = (1.0 / 0.0181);
        public static final double HoodCancoder_GEAR_RATIO_TOMotor = (1.0 / 0.0956);

        public static final double Flywheel_GEAR_RATIO = 1.0 / (12.0 / 16.0);

        public static final class TriggerConstants {
                public static final Current TRIGGER_STATOR_CURRENT_LIMIT = Amp.of(40);
                public static final Current TRIGGER_SUPPLY_CURRENT_LIMIT = Amp.of(45);
        }
}
