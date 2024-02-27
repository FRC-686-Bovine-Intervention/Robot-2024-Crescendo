package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class MathExtraUtil {
    public static double dotProduct(Translation2d a, Translation2d b) {
        return a.getX()*b.getX() + a.getY()*b.getY();
    }
}
