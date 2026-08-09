package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Constants.IDs.DriveConstants;
import frc.robot.subsystems.superstructure;
import frc.robot.util.FIeldHelper.AllianceFlipUtil;

public class DrivetrainCmd extends Command {
    private final CommandSwerveDrivetrain drive;
    private final CommandXboxController joystick;

    // 建構子 (Constructor)
    public DrivetrainCmd(CommandSwerveDrivetrain drive, CommandXboxController joystick) {
        this.drive = drive;
        this.joystick = joystick;

        // 宣告這個 Command 需要佔用底盤子系統
        addRequirements(drive);
    }

@Override
    public void execute() {
        // 1. 讀取原始搖桿 X, Y 輸入
        double xSpeed = MathUtil.applyDeadband(-joystick.getLeftY(), 0.05) * DriveConstants.kMaxSpeedMeterPerSecond;
        double ySpeed = MathUtil.applyDeadband(-joystick.getLeftX(), 0.05) * DriveConstants.kMaxSpeedMeterPerSecond;

        // ✨ 2. 紅藍方視角反轉：只負責翻轉方向，算完就關閉括號！
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
            xSpeed = -xSpeed;
            ySpeed = -ySpeed;
        }

        // 3. 讀取右搖桿旋轉輸入 (不管紅藍方，旋轉邏輯都一樣)
        double rotSpeed = MathUtil.applyDeadband(-joystick.getRightX() * 0.8, 0.05)
                * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

        // 4. 將速度送給底盤 (不管紅藍方，每一圈都必須執行這行)
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        drive.runVelocity(fieldSpeeds);
    }
}