package frc.robot.subsystems.rollers;

import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NoteVisualizer;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.kicker.Kicker;
import frc.robot.util.EdgeDetector;

public class Rollers extends SubsystemBase {
    private final RollerSensorsIO sensorsIO;
    private final RollerSensorsIOInputsAutoLogged inputs = new RollerSensorsIOInputsAutoLogged();

    private final EdgeDetector kickerEdgeDetector = new EdgeDetector(() -> inputs.kickerSensor);

    private final Intake intake;
    private final Kicker kicker;

    public Rollers(Intake intake, Kicker kicker, RollerSensorsIO sensorsIO) {
        System.out.println("[Init Rollers] Instantiating Rollers");
        this.sensorsIO = sensorsIO;
        System.out.println("[Init Rollers] Sensors IO: " + this.sensorsIO.getClass().getSimpleName());
        this.intake = intake;
        this.kicker = kicker;
        SmartDashboard.putData("Subsystems/Rollers", this);
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

    @AutoLogOutput(key = "Rollers/Goal")
    public Goal goal = Goal.IDLE;

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

    @Override
    public void periodic() {
        sensorsIO.updateInputs(inputs);
        Logger.processInputs("RollerSensors", inputs);
        kickerEdgeDetector.update();
        if(inputs.intakeSensor) {
            gamePiece = Optional.of(GamePieceState.INTAKE);
        }
        if(inputs.kickerSensor) {
            gamePiece = Optional.of(GamePieceState.KICKER);
        }
        if(kickerEdgeDetector.fallingEdge()) {
            gamePiece = Optional.empty();
        }
        NoteVisualizer.internalNote = gamePiece;
        goal.runGoal(intake, kicker);
        intake.periodic();
        kicker.periodic();
    }

    public Command setGoalCommand(Goal goal) {
        return startEnd(
            () -> this.goal = goal,
            () -> this.goal = Goal.IDLE
        )
        .withName("Rollers " + goal.name());
    }
    public Command setIntakeGoalCommand(Intake.Goal goal) {
        return defer(() -> setGoalCommand(Goal.from(goal, this.goal.kickerGoal)));
    }
    public Command setKickerGoalCommand(Kicker.Goal goal) {
        return defer(() -> setGoalCommand(Goal.from(goal, this.goal.intakeGoal)));
    }
}
