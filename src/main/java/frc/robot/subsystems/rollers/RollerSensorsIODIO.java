package frc.robot.subsystems.rollers;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import frc.robot.Constants.DIOPorts;

public class RollerSensorsIODIO implements RollerSensorsIO {
    private final DigitalInput intakeSensor = new DigitalInput(DIOPorts.intakeSensorPort);
    private final DigitalInput kickerSensor = new DigitalInput(DIOPorts.kickerSensorPort);

    private final Queue<Boolean> intakeHistory = new ArrayBlockingQueue<Boolean>((int)Math.ceil(Constants.dtSeconds / Constants.sensorDtSeconds) + 2);
    private final Queue<Boolean> kickerHistory = new ArrayBlockingQueue<Boolean>((int)Math.ceil(Constants.dtSeconds / Constants.sensorDtSeconds) + 2);

    private final Notifier thread = new Notifier(() -> {
        synchronized(this) {
            intakeHistory.offer(!this.intakeSensor.get());
            kickerHistory.offer(!this.kickerSensor.get());
        }
    });

    public RollerSensorsIODIO() {
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
