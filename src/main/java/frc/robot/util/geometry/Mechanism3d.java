package frc.robot.util.geometry;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public final class Mechanism3d {
    private final Pose3d origin;
    private final Map<String, MechanismRoot3d> roots;
    
    public Mechanism3d() {
        origin = new Pose3d();
        roots = new HashMap<>();
    }

    public MechanismRoot3d getRoot(String name, double x, double y, double z) {
        MechanismRoot3d existing = roots.get(name);
        if (existing != null) return existing;

        MechanismRoot3d root = new MechanismRoot3d(name, x, y, z);
        roots.put(name, root);
        return root;
    }

    public Transform3d getTransformTo(MechanismRoot3d root) {
        if (!roots.containsKey(root.getName())) throw new UnsupportedOperationException("Mechanism root: " + root.getName() + " is not part of the Mechanism3d!");
        return new Transform3d(origin, root.getPosition());
    }

    public Transform3d getTransformTo(MechanismObject3d object) {
        return new Transform3d(origin,getObjectPosition(object.getPath()));
    }

    public Transform3d getTransformBetween(MechanismRoot3d from, MechanismRoot3d to) {
        if (!roots.containsKey(from.getName())) throw new UnsupportedOperationException("Mechanism root: " + from.getName() + " is not part of the Mechanism3d!");
        if (!roots.containsKey(to.getName())) throw new UnsupportedOperationException("Mechanism root: " + to.getName() + " is not part of the Mechanism3d!");
        return new Transform3d(from.getPosition(), to.getPosition());
    }

    public Transform3d getTransformBetween(MechanismRoot3d from, MechanismObject3d to) {
        if (!roots.containsKey(from.getName())) throw new UnsupportedOperationException("Mechanism root: " + from.getName() + " is not part of the Mechanism3d!");
        return new Transform3d(from.getPosition(), getObjectPosition(to.getPath()));
    }

    public Transform3d getTransformBetween(MechanismObject3d from, MechanismRoot3d to) {
        if (!roots.containsKey(to.getName())) throw new UnsupportedOperationException("Mechanism root: " + to.getName() + " is not part of the Mechanism3d!");
        return new Transform3d(getObjectPosition(from.getPath()), to.getPosition());
    }

    public Transform3d getTransformBetween(MechanismObject3d from, MechanismObject3d to) {
        return new Transform3d(getObjectPosition(from.getPath()), getObjectPosition(to.getPath()));
    }

    private Pose3d getObjectPosition(String objectPath) {
        String[] pathFragments = objectPath.split("/");
        if (pathFragments.length < 2) {
            throw new IllegalArgumentException("Invalid object path format.");
        }
        String rootName = pathFragments[0];
        if (!roots.containsKey(rootName)) throw new UnsupportedOperationException("Mechanism root: " + rootName + " is not part of the Mechanism3d!");
        MechanismRoot3d root = roots.get(rootName);
        Pose3d position = root.getPosition();
        MechanismObject3d object = root.getObject(pathFragments[1]);
        position = position.transformBy(object.getRelativePosition());
        for (int i = 2; i < pathFragments.length; i++) {
            String objectName = pathFragments[i];
            object = object.getObject(objectName);
            if (object == null) {
                throw new IllegalArgumentException("Mechanism object '" + objectPath + "' not part of the Mechanism3d!.");
            }
            position = position.transformBy(object.getRelativePosition());
        }
        return position;
    }
}
