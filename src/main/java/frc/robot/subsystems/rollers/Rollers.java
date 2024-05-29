package frc.robot.subsystems.rollers;

import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NoteVisualizer;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.kicker.Kicker;
import frc.robot.util.SuppliedEdgeDetector;
import frc.robot.util.VirtualSubsystem;

public class Rollers extends VirtualSubsystem {
    private final RollerSensorsIO sensorsIO;
    private final RollerSensorsIOInputsAutoLogged inputs = new RollerSensorsIOInputsAutoLogged();

    private final SuppliedEdgeDetector intakeEdgeDetector = new SuppliedEdgeDetector(() -> inputs.intakeSensorHistory);
    private final SuppliedEdgeDetector kickerEdgeDetector = new SuppliedEdgeDetector(() -> inputs.kickerSensorHistory);

    public final Intake intake;
    public final Kicker kicker;

    public Rollers(Intake intake, Kicker kicker, RollerSensorsIO sensorsIO) {
        System.out.println("[Init Rollers] Instantiating Rollers");
        this.sensorsIO = sensorsIO;
        System.out.println("[Init Rollers] Sensors IO: " + this.sensorsIO.getClass().getSimpleName());
        this.intake = intake;
        this.kicker = kicker;
    }

    public static enum Goal {
        IDLE(
            Intake.Goal.IDLE,
            Kicker.Goal.IDLE
        ),
        ANTI_DEADZONE(
            Intake.Goal.ANTI_DEADZONE,
            Kicker.Goal.ANTI_DEADZONE
        ),
        INTAKE(
            Intake.Goal.INTAKE,
            Kicker.Goal.IDLE
        ),
        FEED(
            Intake.Goal.FEED,
            Kicker.Goal.FEED
        ),
        KICK(
            Intake.Goal.IDLE,
            Kicker.Goal.KICK
        ),
        IN_N_OUT(
            Intake.Goal.INTAKE,
            Kicker.Goal.KICK
        ),
        EJECT(
            Intake.Goal.EJECT,
            Kicker.Goal.EJECT
        ),
        ;
        public final Intake.Goal intakeGoal;
        public final Kicker.Goal kickerGoal;
        Goal(Intake.Goal intakeGoal, Kicker.Goal kickerGoal) {
            this.intakeGoal = intakeGoal;
            this.kickerGoal = kickerGoal;
        }

        public void runGoal(Intake intake, Kicker kicker) {
            intake.setGoal(intakeGoal);
            kicker.setGoal(kickerGoal);
        }

        public static Goal from(Intake.Goal intakeGoal, Kicker.Goal kickerGoal) {
            return 
                Arrays.stream(values())
                .filter((g) -> g.intakeGoal == intakeGoal && g.kickerGoal == kickerGoal)
                .findAny()
                .orElseGet(() -> switch (intakeGoal) {
                    default -> Goal.IDLE;
                    case FEED -> Goal.FEED;
                    case EJECT -> Goal.EJECT;
                    case INTAKE -> Goal.INTAKE;
                })
            ;
        }
        public static Goal from(Kicker.Goal kickerGoal, Intake.Goal intakeGoal) {
            return 
                Arrays.stream(values())
                .filter((g) -> g.intakeGoal == intakeGoal && g.kickerGoal == kickerGoal)
                .findAny()
                .orElseGet(() -> switch (kickerGoal) {
                    default -> Goal.IDLE;
                    case FEED -> Goal.FEED;
                    case EJECT -> Goal.EJECT;
                    case KICK -> Goal.KICK;
                })
            ;
        }
    }

    public static enum GamePieceState {
        INTAKE,
        KICKER,
        ;
    }

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
    public boolean kickerFallingEdge() {
        return kickerEdgeDetector.fallingEdge();
    }

    @Override
    public void periodic() {
        sensorsIO.updateInputs(inputs);
        Logger.processInputs("RollerSensors", inputs);
        intakeEdgeDetector.update();
        kickerEdgeDetector.update();
        if(intakeEdgeDetector.getValue()) {
            gamePiece = Optional.of(GamePieceState.INTAKE);
        }
        if(kickerEdgeDetector.fallingEdge()) {
            gamePiece = Optional.empty();
        }
        if(kickerEdgeDetector.getValue()) {
            gamePiece = Optional.of(GamePieceState.KICKER);
        }
        NoteVisualizer.internalNote = gamePiece;
        intake.periodic();
        kicker.periodic();
        Leds.getInstance().noteSecured.set(noteInKicker());
    }

    public Command setGoalCommand(Goal goal) {
        return Commands.parallel(
            setIntakeGoalCommand(goal.intakeGoal),
            setKickerGoalCommand(goal.kickerGoal)
        )
        .withName("Rollers " + goal.name());
    }
    public Command setIntakeGoalCommand(Intake.Goal goal) {
        return intake.setGoalCommand(goal);
    }
    public Command setKickerGoalCommand(Kicker.Goal goal) {
        return kicker.setGoalCommand(goal);
    }
}
