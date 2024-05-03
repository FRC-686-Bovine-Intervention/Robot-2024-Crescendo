package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSensorsIO {
    @AutoLog
    public static class RollerSensorsIOInputs {
        boolean[] intakeSensorHistory;
        boolean[] kickerSensorHistory;
    }

    public default void updateInputs(RollerSensorsIOInputs inputs) {}
}
