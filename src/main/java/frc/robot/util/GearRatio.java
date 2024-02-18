package frc.robot.util;

public class GearRatio {
    private final double drivenToDriveRatio;
    private final double driveToDrivenRatio;

    private GearRatio(double ratio) {
        this.drivenToDriveRatio = ratio;
        this.driveToDrivenRatio = 1/ratio;
    }

    public static GearRatio start(int toothCount) {
        return new GearRatio(toothCount);
    }
    public GearRatio driven(int toothCount) {
        return new GearRatio(drivenToDriveRatio * toothCount);
    }
    public GearRatio drive(int toothCount) {
        return new GearRatio(drivenToDriveRatio / toothCount);
    }
    public GearRatio andThen(GearRatio other) {
        return new GearRatio(drivenToDriveRatio * other.drivenToDriveRatio);
    }
    public GearRatio inverse() {
        return new GearRatio(driveToDrivenRatio);
    }
    public double drivenToDrive() {
        return drivenToDriveRatio;
    }
    public double driveToDriven() {
        return driveToDrivenRatio;
    }
}
