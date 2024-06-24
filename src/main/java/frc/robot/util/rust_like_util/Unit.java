package frc.robot.util.rust_like_util;

public class Unit {
    private Unit() {}

    public static final Unit unit = new Unit();

    @Override
    public String toString() {
        return "()";
    }
}
