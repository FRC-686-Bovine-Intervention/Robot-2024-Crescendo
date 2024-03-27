package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.loggerUtil.LoggedMotor;

public interface ClimberIO {
    
    @AutoLog
    public class ClimberIOInputs {
        public LoggedMotor climberMotor = new LoggedMotor();
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void stop() {}
}
