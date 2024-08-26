package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.rollers.Rollers.GamePieceState;

public class NoteVisualizer {
    private final static Map<GamePieceState, Transform3d> toNoteMap = new HashMap<>();
    static {
        toNoteMap.put(
            GamePieceState.INTAKE,
            new Transform3d(
                new Translation3d(
                    -0.25,
                    +0,
                    +0.2
                ),
                new Rotation3d(
                    +0,
                    +Math.PI/2,
                    +0
                )
            )
        );
        toNoteMap.put(
            GamePieceState.KICKER,
            new Transform3d(
                new Translation3d(
                    Meters.of(0.2),
                    Inches.of(0),
                    Inches.of(2.875)
                ),
                new Rotation3d(
                    +0,
                    +0,
                    +0
                )
            )
        );
    }

    public static Optional<GamePieceState> internalNote = Optional.empty();
    public static Transform3d robotToPivot = new Transform3d();
    
    public static Pose3d[] logInternal() {
        return internalNote.map((note) -> new Pose3d[]{
            new Pose3d(RobotState.getInstance().getPose()).transformBy(
                switch(note) {
                    default -> toNoteMap.get(note);
                    case KICKER -> robotToPivot.plus(toNoteMap.get(note));
                }
            )
        }).orElse(new Pose3d[]{});
    }
}
