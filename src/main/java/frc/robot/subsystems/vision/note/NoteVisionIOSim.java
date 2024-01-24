package frc.robot.subsystems.vision.note;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.note.NoteVision.TrackedNote;

public class NoteVisionIOSim implements NoteVisionIO {
    @Override
    public void updateInputs(NoteVisionIOInputs inputs) {
        inputs.trackedNotes = new TrackedNote[] {
            new TrackedNote(
                new Translation2d(
                    1, 
                    5
                ),
                Math.PI
            ),
            new TrackedNote(
                new Translation2d(
                    5, 
                    4
                ),
                Math.E
            ),
            new TrackedNote(
                new Translation2d(
                    8, 
                    3
                ),
                5
            ),
        };
    }
}
