package frc.robot.subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.google.gson.annotations.Until;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterConstants;

public abstract class TurretIO {

    public enum ShootState {
        ACTIVE_SHOOTING,
        TRACKING
    }

    public double lastSetpointRads = 0.0;

    public abstract Angle getAnglegoal();

    public abstract void setAngle(
            Rotation2d robotHeading,
            Angle targetRad,
            ShootState state,
            double chassisOmegaRadsPerSec, // 底盤的旋轉速度 (Rad/s)
            double fieldVelocityX, // 底盤的 X 軸平移速度 (m/s)
            double fieldVelocityY, // 底盤的 Y 軸平移速度 (m/s)
            double deltaX, // 機器人到目標的 X 距離 (m)
            double deltaY // 機器人到目標的 Y 距離 (m)
    );

    // public abstract void resetAngle();

    public abstract Angle getAngle();

    public abstract boolean isAtSetPosition();

    public abstract Command sysid();

    public Angle calculate(Rotation2d robotHeading, Angle targetRad, ShootState state) {

        lastSetpointRads = this.getAngle().baseUnitMagnitude();

        double currentMin, currentMax;

        if (state == ShootState.ACTIVE_SHOOTING) {
            currentMin = ShooterConstants.HARD_MIN_LIMIT;
            currentMax = ShooterConstants.HARD_MAX_LIMIT;
        } else {
            currentMin = ShooterConstants.SOFT_MIN_LIMIT;
            currentMax = ShooterConstants.SOFT_MAX_LIMIT;
        }

        // 🟢 既然傳進來的 targetRad 已經是「相對車頭角度」，我們直接標準化它即可
        // 使用 Rotation2d 是為了把角度漂亮地收斂到 [-PI, PI] 之間，方便後續找最佳圈數
        Rotation2d relativeGoal = Rotation2d.fromRadians(targetRad.in(Radians));
        double baseAngleRads = relativeGoal.getRadians();

        double bestAngle = 0.0;
        boolean foundValidAngle = false;

        // 尋找在硬體/軟體極限內，距離當前砲塔位置最近的合法「圈數」 (-2圈 到 +2圈)
        for (int i = -2; i <= 2; i++) {
            double candidate = baseAngleRads + (Math.PI * 2.0 * i);

            if (candidate >= currentMin && candidate <= currentMax) {

                if (!foundValidAngle) {
                    bestAngle = candidate;
                    foundValidAngle = true;
                } else {
                    if (Math.abs(lastSetpointRads - candidate) < Math
                            .abs(lastSetpointRads - bestAngle)) {
                        bestAngle = candidate;
                    }
                }
            }
        }

        if (!foundValidAngle && state == ShootState.TRACKING) {
            return calculate(robotHeading, targetRad, ShootState.ACTIVE_SHOOTING);
        }

        if (!foundValidAngle) {
            return Radians.of(lastSetpointRads);
        }

        lastSetpointRads = bestAngle;
        return Radians.of(bestAngle);
    }
}