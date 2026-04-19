package frc.robot.util.MathHelper;

public class RobustCRTCalculator {

    public static double calculateAbsolutePosition(
        PositionWithGearRatio master, 
        PositionWithGearRatio slave
        ) {

        double diff = master.position - slave.position;
        
        diff %= 1.0;
        if (diff < 0) diff += 1.0;

        int k = (int) Math.round(diff * slave.gearRatio);

        double absolutePos = (k + master.position) / master.gearRatio;
        
        if (absolutePos >= 1.0) absolutePos -= 1.0;
        else if (absolutePos < 0) absolutePos += 1.0;

        return absolutePos;
    }
}
