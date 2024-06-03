package frc.robot.util.geometry;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Transform3d;

public abstract class MechanismObject3d {
    private final String name;
    private String path;
    private final Map<String, MechanismObject3d> objects = new HashMap<>();
    
    protected MechanismObject3d(String name) {
        this.name = name;
    }

    public <T extends MechanismObject3d> T append(T object) {
        if (objects.containsKey(object.getName())) throw new UnsupportedOperationException("Mechanism object names must be unique!");
        object.setParentPath(name);
        objects.put(object.getName(), object);
        return object;
    }

    void setParentPath(String parentPath) {
        this.path = parentPath + "/" + name;
    }

    String getPath() {
        return path;
    }

    public String getName() {
        return name;
    }

    public MechanismObject3d getObject(String name) {
        return objects.get(name);
    }

    abstract Transform3d getRelativePosition();
}
