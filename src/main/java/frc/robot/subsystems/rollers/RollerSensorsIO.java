package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSensorsIO {
    @AutoLog
    public static class RollerSensorsIOInputs {
        boolean intakeSensor;
        boolean kickerSensor;
    }

    public default void updateInputs(RollerSensorsIOInputs inputs) {}
}
