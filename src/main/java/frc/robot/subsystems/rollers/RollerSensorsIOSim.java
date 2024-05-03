package frc.robot.subsystems.rollers;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.BooleanSupplier;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;

public class RollerSensorsIOSim implements RollerSensorsIO {
    private final BooleanSupplier intakeSensor;
    private final BooleanSupplier kickerSensor;

    private final Queue<Boolean> intakeHistory = new ArrayBlockingQueue<Boolean>((int)Math.ceil(Constants.dtSeconds / Constants.sensorDtSeconds) + 2);
    private final Queue<Boolean> kickerHistory = new ArrayBlockingQueue<Boolean>((int)Math.ceil(Constants.dtSeconds / Constants.sensorDtSeconds) + 2);

    private final Notifier thread;

    public RollerSensorsIOSim(BooleanSupplier intakeSensor, BooleanSupplier kickerSensor) {
        this.intakeSensor = intakeSensor;
        this.kickerSensor = kickerSensor;
        thread = new Notifier(() -> {
            synchronized(this) {
                intakeHistory.offer(this.intakeSensor.getAsBoolean());
                kickerHistory.offer(this.kickerSensor.getAsBoolean());
            }
        });
        thread.startPeriodic(Constants.sensorDtSeconds);
    }

    @Override
    public synchronized void updateInputs(RollerSensorsIOInputs inputs) {
        inputs.intakeSensorHistory = new boolean[intakeHistory.size()];
        final var intHistory = intakeHistory.toArray(Boolean[]::new);
        IntStream.range(0, intHistory.length).forEach((i) -> inputs.intakeSensorHistory[i] = intHistory[i]);
        intakeHistory.clear();

        inputs.kickerSensorHistory = new boolean[kickerHistory.size()];
        final var kickHistory = kickerHistory.toArray(Boolean[]::new);
        IntStream.range(0, kickHistory.length).forEach((i) -> inputs.kickerSensorHistory[i] = kickHistory[i]);
        kickerHistory.clear();
    }
}
