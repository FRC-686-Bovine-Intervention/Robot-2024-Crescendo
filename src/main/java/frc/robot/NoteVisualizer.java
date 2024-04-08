package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class NoteVisualizer {
    public enum InternalNote {
        Intake(
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
        ),
        Kicker(
            new Transform3d(
                new Translation3d(
                    Meters.of(0.2),
                    Inches.of(0),
                    Inches.of(3)
                ),
                new Rotation3d(
                    +0,
                    +0,
                    +0
                )
            )
        ),
        ;
        private Transform3d robotToNote;
        InternalNote(Transform3d robotToNote) {
            this.robotToNote = robotToNote;
        }
    }

    public static Optional<InternalNote> internalNote = Optional.empty();
    public static Transform3d robotToPivot = new Transform3d();
    
    public static Pose3d[] logInternal() {
        return internalNote.map((note) -> new Pose3d[]{
            new Pose3d(RobotState.getInstance().getPose()).transformBy(
                switch(note) {
                    default -> note.robotToNote;
                    case Kicker -> robotToPivot.plus(note.robotToNote);
                }
            )
        }).orElse(new Pose3d[]{});
    }

    public static void setInternalNote(InternalNote note) {
        internalNote = Optional.ofNullable(note);
    }
}
