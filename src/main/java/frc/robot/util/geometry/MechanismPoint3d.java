package frc.robot.util.geometry;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class MechanismPoint3d extends MechanismObject3d {
    private Transform3d relativePosition;

    public MechanismPoint3d(String name, double x, double y, double z) {
        super(name);
        setPosition(x, y, z);
    }

    @Override
    Transform3d getRelativePosition() {
        return relativePosition;
    }

    public void setPosition(double x, double y, double z) {
        relativePosition = new Transform3d(new Translation3d(x, y, z), new Rotation3d());
    }
    
}
