package frc.robot.subsystems.manualOverrides;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DIOPorts;

public class ManualOverrides {
    private final DigitalInput pivotCoastDIO = new DigitalInput(DIOPorts.redButtonPort);

    public final Trigger redButton = new Trigger(() -> !pivotCoastDIO.get());

    public ManualOverrides() {}
}
