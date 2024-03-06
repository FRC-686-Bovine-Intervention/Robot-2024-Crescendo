package frc.robot.subsystems.shooter.amp;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.loggerUtil.LoggedMotor;

public interface AmpIO {
    
    @AutoLog
    public static class AmpIOInputs {
        public LoggedMotor ampMotor = new LoggedMotor();
    }
    
    public default void updateInputs(AmpIOInputs inputs) {}

    public default void setVoltage(double volts) {}
}
