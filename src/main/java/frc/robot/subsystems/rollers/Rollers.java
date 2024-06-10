package frc.robot.subsystems.rollers;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.NoteVisualizer;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.kicker.Kicker;
import frc.robot.util.EdgeDetector;
import frc.robot.util.SuppliedEdgeDetector;
import frc.robot.util.VirtualSubsystem;

public class Rollers extends VirtualSubsystem {
    private final RollerSensorsIO sensorsIO;
    private final RollerSensorsIOInputsAutoLogged inputs = new RollerSensorsIOInputsAutoLogged();

    private final EdgeDetector intakeEdgeDetector = new EdgeDetector();
    private final EdgeDetector kickerEdgeDetector = new EdgeDetector();

    public final Intake intake;
    public final Kicker kicker; 

    public Rollers(Intake intake, Kicker kicker, RollerSensorsIO sensorsIO) {
        System.out.println("[Init Rollers] Instantiating Rollers");
        this.sensorsIO = sensorsIO;
        System.out.println("[Init Rollers] Sensors IO: " + this.sensorsIO.getClass().getSimpleName());
        this.intake = intake;
        this.kicker = kicker;
    }

    public Command antiDeadzone() {
        return Commands.parallel(intake.antiDeadzone(), kicker.antiDeadZone());
    }

    public Command intake() {
        return Commands.parallel(intake.intake(), kicker.idle());
    }

    public Command feed() {
        return Commands.parallel(intake.feed(), kicker.feed());
    }

    public Command kick() {
        return Commands.parallel(intake.idle(), kicker.kick());
    }

    public Command inNOut() {
        return Commands.parallel(intake.intake(), kicker.kick());
    }

    public Command eject() {
        return Commands.parallel(intake.eject(), kicker.eject());
    }
    
    public Command idle() {
        return Commands.parallel(intake.idle(), kicker.idle());
    }

    public static enum GamePieceState {
        INTAKE,
        KICKER,
        ;
    }

    private final SuppliedEdgeDetector noteExitDetector = new SuppliedEdgeDetector(this::noNote);
    public Optional<GamePieceState> gamePiece = Optional.empty();
    public boolean noNote() {
        return gamePiece.isEmpty();
    }
    public boolean noteInIntake() {
        return gamePiece.equals(Optional.of(GamePieceState.INTAKE));
    }
    public boolean noteInKicker() {
        return gamePiece.equals(Optional.of(GamePieceState.KICKER));
    }
    public boolean noteExited() {
        return noteExitDetector.risingEdge();
    }

    public Trigger isKicking() {
        return kicker.isKicking;
    }

    @Override
    public void periodic() {
        sensorsIO.updateInputs(inputs);
        Logger.processInputs("RollerSensors", inputs);
        for(int i = 0; i < Math.min(inputs.intakeSensorHistory.length, inputs.kickerSensorHistory.length); i++) {
            intakeEdgeDetector.update(inputs.intakeSensorHistory[i]);
            kickerEdgeDetector.update(inputs.kickerSensorHistory[i]);
            if(intakeEdgeDetector.getValue()) {
                gamePiece = Optional.of(GamePieceState.INTAKE);
            }
            if(kickerEdgeDetector.fallingEdge()) {
                gamePiece = Optional.empty();
            }
            if(kickerEdgeDetector.getValue()) {
                gamePiece = Optional.of(GamePieceState.KICKER);
            }
        }
        noteExitDetector.update();
        NoteVisualizer.internalNote = gamePiece;
        intake.periodic();
        kicker.periodic();
        Leds.getInstance().noteSecured.set(noteInKicker());
    }
}
