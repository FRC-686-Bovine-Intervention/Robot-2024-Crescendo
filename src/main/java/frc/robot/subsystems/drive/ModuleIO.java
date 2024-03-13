package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.loggerUtil.LoggedEncoder;
import frc.robot.util.loggerUtil.LoggedMotor;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {
        public double driveSetpointRadPerSec = 0.0;
        public LoggedMotor driveMotor = new LoggedMotor();
        public LoggedEncoder driveEncoder = new LoggedEncoder();

        public double turnAbsolutePositionRad = 0.0;
        public LoggedMotor turnMotor = new LoggedMotor();
        public LoggedEncoder turnEncoder = new LoggedEncoder();
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified voltage. */
    public default void setDriveVoltage(double volts) {}

    /** Run the turn motor at the specified voltage. */
    public default void setTurnVoltage(double volts) {}

    /** Enable or disable brake mode on the drive motor. */
    public default void setDriveBrakeMode(Boolean enable) {}

    /** Enable or disable brake mode on the turn motor. */
    public default void setTurnBrakeMode(Boolean enable) {}

    public default void stop() {}

    /** Zero drive encoders */
    public default void zeroEncoders() {}
}
