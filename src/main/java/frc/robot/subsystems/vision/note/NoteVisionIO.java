package frc.robot.subsystems.vision.note;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.vision.note.NoteVision.TrackedNote;

public interface NoteVisionIO {
    @AutoLog
    public static class NoteVisionIOInputs {
        public boolean connected;
        public TrackedNote[] trackedNotes = new TrackedNote[0];
    }

    public default void updateInputs(NoteVisionIOInputs inputs) {}
}
