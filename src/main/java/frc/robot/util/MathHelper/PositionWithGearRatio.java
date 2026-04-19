package frc.robot.util.MathHelper;

public class PositionWithGearRatio {

    public final double position;
    public final int gearRatio;

    /**
     * @param position  當前編碼器位置
     * @param gearRatio 物理齒數
     */
    public PositionWithGearRatio(double position, int gearRatio) {
        this.position = position;
        this.gearRatio = gearRatio;
    }
}
