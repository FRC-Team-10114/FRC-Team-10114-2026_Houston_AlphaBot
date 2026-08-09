package frc.robot.util.Swerve;
//簡單的宣告
//record 是一種 簡化的類別（class），專門用來儲存資料（資料容器）。也叫做「資料記錄型態」
public record ModuleLimits(
    double maxDriveVelocity, double maxDriveAcceleration, double maxSteeringVelocity) {}