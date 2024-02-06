package frc.robot.util;

public class GearRatio {
    private final double ratio;

    private GearRatio(double ratio) {
        this.ratio = ratio;
    }

    public static GearRatio start(int toothCount) {
        return new GearRatio(toothCount);
    }
    public GearRatio driven(int toothCount) {
        return new GearRatio(ratio * toothCount);
    }
    public GearRatio drive(int toothCount) {
        return new GearRatio(ratio / toothCount);
    }
    public double drivenToDrive() {
        return ratio;
    }
    public double driveToDriven() {
        return 1/ratio;
    }
}
