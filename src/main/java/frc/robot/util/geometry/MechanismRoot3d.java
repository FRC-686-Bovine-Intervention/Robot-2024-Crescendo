package frc.robot.util.geometry;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class MechanismRoot3d {
    private final String name;
    private final Map<String, MechanismObject3d> objects;
    private Pose3d position;

    public MechanismRoot3d(String name, double x, double y, double z) {
        this.objects = new HashMap<>();
        this.name = name;
        setPosition(x, y, z);
    }

    public <T extends MechanismObject3d> T append(T object) {
        if (objects.containsKey(object.getName())) throw new UnsupportedOperationException("Mechanism object names must be unique!");
        object.setParentPath(name);
        objects.put(object.getName(), object);
        return object;
    }

    public void setPosition(double x, double y, double z) {
        position = new Pose3d(new Translation3d(x, y, z), new Rotation3d());
    }

    public String getName() {
        return name;
    }

    public Pose3d getPosition() {
        return position;
    }

    public MechanismObject3d getObject(String name) {
        if (!objects.containsKey(name)) throw new UnsupportedOperationException("Mechanism object is not part of the Mechanism3d!");
        return objects.get(name);
    }
}
