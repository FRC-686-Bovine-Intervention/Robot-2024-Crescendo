package frc.robot.subsystems.rollers;

import java.util.function.BooleanSupplier;

public class RollerSensorsIOSim implements RollerSensorsIO {
    private final BooleanSupplier intakeSensor;
    private final BooleanSupplier kickerSensor;

    public RollerSensorsIOSim(BooleanSupplier intakeSensor, BooleanSupplier kickerSensor) {
        this.intakeSensor = intakeSensor;
        this.kickerSensor = kickerSensor;
    }

    @Override
    public void updateInputs(RollerSensorsIOInputs inputs) {
        inputs.intakeSensor = intakeSensor.getAsBoolean();
        inputs.kickerSensor = kickerSensor.getAsBoolean();
    }
}
