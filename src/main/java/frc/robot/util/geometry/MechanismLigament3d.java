package frc.robot.util.geometry;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class MechanismLigament3d extends MechanismObject3d {
    private double length;
    private double pitchAngle;
    private double yawAngle;
    private double rollAngle;

    public MechanismLigament3d(String name, double length, double rollAngle, double pitchAngle, double yawAngle) {
        super(name);
        setLength(length);
        setRollAngle(rollAngle);
        setPitchAngle(pitchAngle);
        setYawAngle(yawAngle);
    }

    public void setLength(double length) {
        this.length = length;
    }

    public void setRollAngle(double rollAngle) {
        this.rollAngle = rollAngle;
    }

    public void setPitchAngle(double pitchAngle) {
        this.pitchAngle = pitchAngle;
    }

    public void setYawAngle(double yawAngle) {
        this.yawAngle = yawAngle;
    }

    @Override
    public Transform3d getRelativePosition() {
        var x = Math.sin(pitchAngle) * Math.cos(yawAngle);
        var y = Math.sin(pitchAngle) * Math.cos(pitchAngle);
        var z = Math.cos(pitchAngle);
        return new Transform3d(new Translation3d(x, y, z).times(length), new Rotation3d(rollAngle, pitchAngle, yawAngle));
    }
}