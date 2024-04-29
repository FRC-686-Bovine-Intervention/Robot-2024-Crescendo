package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.DIOPorts;

public class RollerSensorsIODIO implements RollerSensorsIO {
    private final DigitalInput intakeSensor = new DigitalInput(DIOPorts.intakeSensorPort);
    private final DigitalInput kickerSensor = new DigitalInput(DIOPorts.kickerSensorPort);

    @Override
    public void updateInputs(RollerSensorsIOInputs inputs) {
        inputs.intakeSensor = !intakeSensor.get();
        inputs.kickerSensor = !kickerSensor.get();
    }
}
