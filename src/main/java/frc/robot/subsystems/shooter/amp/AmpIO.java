package frc.robot.subsystems.shooter.amp;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.loggerUtil.LoggedEncoder;
import frc.robot.util.loggerUtil.LoggedMotor;

public interface AmpIO {
    
    @AutoLog
    public static class AmpIOInputs {
        public LoggedEncoder ampEncoder = new LoggedEncoder();
        public LoggedMotor ampMotor = new LoggedMotor();
    }
    
    public default void updateInputs(AmpIOInputs inputs) {}

    public default void setVoltage(double volts) {}
}
