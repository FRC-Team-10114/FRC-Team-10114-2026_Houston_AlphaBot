package frc.robot.util.RobotEvent;

public class Event {
    public Event() {
    }

    @FunctionalInterface
    public interface NeedResetPoseEvent {
        void NeedResetPose();
    }

    @FunctionalInterface
    public interface TargetInactive {
        void TargetInactive();
    }

    @FunctionalInterface
    public interface Targetactive {
        void Targetactive();
    }

    @FunctionalInterface
    public interface ShootingStateTrue {
        void ShootingStateTrue();
    }

    @FunctionalInterface
    public interface ShootingStateFalse {
        void ShootingStateFalse();
    }

    @FunctionalInterface
    public interface InTrench {
        void InTrench();
    }
    @FunctionalInterface
    public interface NotInTrench {
        void NotInTrench();
    }
    public interface LEDRainbow {
        void LEDRainbow();
    }
}