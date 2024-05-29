package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.GameState;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.SuppliedEdgeDetector;
import frc.robot.util.VirtualSubsystem;

public class AutoManager extends VirtualSubsystem {
    private final AutoSelector selector;

    private Command autonomousCommand;
    private final SuppliedEdgeDetector autoEnabled = new SuppliedEdgeDetector(DriverStation::isAutonomousEnabled);
    private final SuppliedEdgeDetector autoScheduled = new SuppliedEdgeDetector(() -> autonomousCommand != null && autonomousCommand.isScheduled());

    public AutoManager(AutoSelector selector) {
        this.selector = selector;
    }

    @Override
    public void periodic() {
        autoEnabled.update();
        autoScheduled.update();
        if(autoEnabled.risingEdge()) {
            autonomousCommand = selector.getSelectedAutoCommand();
            if(autonomousCommand != null) {
                autonomousCommand.asProxy()
                .beforeStarting(
                    () -> GameState.getInstance().AUTONOMOUS_COMMAND_FINISH.clear()
                )
                .finallyDo(
                    (interrupted) -> {
                        GameState.getInstance().AUTONOMOUS_COMMAND_FINISH.set();
                        var autoTime = GameState.getInstance().BEGIN_ENABLE.getTimeSince();
                        if(autoTime > AutoConstants.allottedAutoTime) {
                            System.out.println(String.format("[AutoManager] Autonomous overran the allotted %.1f seconds!", AutoConstants.allottedAutoTime));
                            Leds.getInstance().autonomousOverrun.setCommand().withTimeout(1.5).schedule();
                        }
                        if(interrupted) {
                            System.out.println(String.format("[AutoManager] Autonomous interrupted after %.2f seconds", autoTime));
                        } else {
                            System.out.println(String.format("[AutoManager] Autonomous finished in %.2f seconds", autoTime));
                        }
                    }
                )
                .schedule();
            }
        }
        if(autoEnabled.fallingEdge() && autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }
}
