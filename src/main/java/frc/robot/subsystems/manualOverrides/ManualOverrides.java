package frc.robot.subsystems.manualOverrides;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DIOPorts;
import frc.robot.util.EdgeDetector;
import frc.robot.util.VirtualSubsystem;

public class ManualOverrides extends VirtualSubsystem {
    private final DigitalInput pivotCoastDIO = new DigitalInput(DIOPorts.redButtonPort);
    private final EdgeDetector pivotCoastButton = new EdgeDetector(() -> !pivotCoastDIO.get());

    private final BooleanConsumer pivotSetCoast;

    public ManualOverrides(BooleanConsumer pivotSetCoast) {
        this.pivotSetCoast = pivotSetCoast;
    }

    private boolean pivotToggle;

    @Override
    public void periodic() {
        pivotCoastButton.update();
        if(DriverStation.isEnabled()) {
            pivotToggle = false;
            return;
        }
        if(pivotCoastButton.fallingEdge()) {
            pivotToggle = !pivotToggle;
            if(!pivotToggle) {
                pivotSetCoast.accept(false);
            }
        }
        if(pivotToggle) {
            pivotSetCoast.accept(true);
        }
    }
}
