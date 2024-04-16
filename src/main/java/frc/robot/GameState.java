package frc.robot;

import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.EdgeDetector;

public class GameState {
    private static GameState instance;
    public static GameState getInstance() {if(instance == null) {instance = new GameState();} return instance;}

    public static enum EnabledMode {
        TELEOP(DriverStation::isTeleop),
        AUTONOMOUS(DriverStation::isAutonomous),
        TEST(DriverStation::isTest),
        ;
        private final BooleanSupplier isMode;
        EnabledMode(BooleanSupplier isMode) {
            this.isMode = isMode;
        }
        public boolean isTeleop() {return equals(TELEOP);}
        public boolean isAutonomous() {return equals(AUTONOMOUS);}
        public boolean isTest() {return equals(TEST);}

        public static Optional<EnabledMode> getSelectedMode() {
            return Arrays.stream(values()).filter((m) -> m.isMode.getAsBoolean()).findAny();
        }
    }
    
    public static enum Timestamp {
        BEGIN_ENABLE,
        LAST_ENABLE,
        AUTONOMOUS_COMMAND_FINISH,
        ;
        public OptionalDouble timestamp;
        public double getTimeSince() {
            return getTimeSince(Timer.getFPGATimestamp());
        }
        public double getTimeSince(double time) {
            return timestamp.stream().map((t) -> time - t).findAny().orElse(0);
        }
        public boolean hasBeenSince(double length) {
            return getTimeSince() >= length;
        }

        public void set() {
            set(Timer.getFPGATimestamp());
        }
        public void set(double timestamp) {
            this.timestamp = OptionalDouble.of(timestamp);
        }
        public void clear() {
            this.timestamp = OptionalDouble.empty();
        }
        public boolean isSet() {
            return timestamp.isPresent();
        }
    }

    public final EdgeDetector enabled = new EdgeDetector(DriverStation::isEnabled);
    public Optional<EnabledMode> currentEnabledMode = Optional.empty();
    public EnabledMode lastEnabledMode = EnabledMode.TELEOP;

    public void periodic() {
        currentEnabledMode = EnabledMode.getSelectedMode().filter((m) -> DriverStation.isEnabled());
        currentEnabledMode.ifPresent((m) -> lastEnabledMode = m);
        enabled.update();

        if(enabled.risingEdge()) {
            Timestamp.BEGIN_ENABLE.set();
        }
        if(enabled.fallingEdge()) {
            Timestamp.LAST_ENABLE.set();
        }
    }
}
