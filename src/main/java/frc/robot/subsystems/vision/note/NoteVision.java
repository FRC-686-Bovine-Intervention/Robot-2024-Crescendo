package frc.robot.subsystems.vision.note;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;

public class NoteVision extends VirtualSubsystem {
    private final NoteVisionIO io;
    private final NoteVisionIOInputsAutoLogged inputs = new NoteVisionIOInputsAutoLogged();

    private final ArrayList<TrackedNote> noteMemories = new ArrayList<>();

    private static final LoggedTunableNumber updateDistanceThreshold = new LoggedTunableNumber("Vision/updateDistanceThreshold", 5);
    private static final LoggedTunableNumber posUpdatingFilteringFactor = new LoggedTunableNumber("Vision/posUpdatingFilteringFactor", 0.8);
    private static final LoggedTunableNumber confUpdatingFilteringFactor = new LoggedTunableNumber("Vision/posUpdatingFilteringFactor", 0.5);
    public static final LoggedTunableNumber confidencePerAreaPercent = new LoggedTunableNumber("Vision/confidencePerAreaPercent", 1);
    private static final LoggedTunableNumber confidenceDecayPerSecond = new LoggedTunableNumber("Vision/confidenceDecayPerSecond", 1);

    public NoteVision(NoteVisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("NoteVision", inputs);
        var frameTargets = Arrays.asList(inputs.trackedNotes);
        var connections = new ArrayList<PhotonMemoryConnection>();
        noteMemories.forEach(
            (memory) -> frameTargets.forEach(
                (target) -> {
                    if(memory.fieldPos.getDistance(target.fieldPos) < updateDistanceThreshold.get()) {
                        connections.add(new PhotonMemoryConnection(memory, target));
                    }
                }
            )
        );
        connections.sort((a, b) -> (int) Math.signum(a.getDistance() - b.getDistance()));
        var unusedMemories = new ArrayList<>(noteMemories);
        var unusedTargets = new ArrayList<>(frameTargets);
        while(!connections.isEmpty()) {
            var confirmedConnection = connections.get(0);
            confirmedConnection.memory.updatePosWithFiltering(confirmedConnection.photonFrameTarget);
            confirmedConnection.memory.updateConfidence();
            unusedMemories.remove(confirmedConnection.memory);
            unusedTargets.remove(confirmedConnection.photonFrameTarget);
            connections.removeIf((connection) -> 
                connection.memory == confirmedConnection.memory
                ||
                connection.photonFrameTarget == confirmedConnection.photonFrameTarget
            );
        }
        unusedMemories.forEach((memory) -> {
            if(RobotState.getInstance().getPose().getTranslation().getDistance(memory.fieldPos) > 1) {
                memory.decayConfidence(memory.isWithinView() ? 10 : 1);
            }
        });
        unusedTargets.forEach((target) -> noteMemories.add(target));
        noteMemories.removeIf((memory) -> memory.confidence <= 0);
        noteMemories.removeIf((memory) -> Double.isNaN(memory.fieldPos.getX()));

        // Logger.recordOutput("Vision/Note/Photon Frame Targets", frameTargets.stream().map(NoteVision::targetToPose).toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Note/Note Memories", noteMemories.stream().map(NoteVision::targetToPose).toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Note/Note Confidence", noteMemories.stream().mapToDouble((note) -> note.confidence).toArray());
    }

    public List<TrackedNote> getTrackedNotes() {
        return noteMemories;
    }

    public void forgetNote(TrackedNote note) {
        noteMemories.remove(note);
    }

    private static Pose3d targetToPose(TrackedNote note) {
        return new Pose3d(new Translation3d(note.fieldPos.getX(), note.fieldPos.getY(), Units.inchesToMeters(1)), new Rotation3d());
    }

    private static record PhotonMemoryConnection(TrackedNote memory, TrackedNote photonFrameTarget) {
        public double getDistance() {
            return memory.fieldPos.getDistance(photonFrameTarget.fieldPos);
        }
    }

    public static class TrackedNote implements StructSerializable {
        public Translation2d fieldPos;
        public double confidence;

        public TrackedNote(Translation2d fieldPos, double confidence) {
            this.fieldPos = fieldPos;
            this.confidence = confidence * confUpdatingFilteringFactor.get();
        }

        public void updateConfidence() {
            confidence += confidence * MathUtil.clamp(1 - confUpdatingFilteringFactor.get(), 0, 1); 
        }

        public void updatePosWithFiltering(TrackedNote newNote) {
            this.fieldPos = fieldPos.interpolate(newNote.fieldPos, posUpdatingFilteringFactor.get());
            this.confidence = newNote.confidence;
        }

        public boolean isWithinView() {
            return false;
                // Math.abs() < FOVYawThreshold.get() &&
                // fieldPos.getDistance(new Pose3d(RobotState.getInstance().getPose()).toPose2d().getTranslation()) < FOVDistanceThreshold.get();
        }

        public void decayConfidence(double rate) {
            this.confidence -= confidenceDecayPerSecond.get() * rate * Constants.dtSeconds;
        }

        public static final TrackedNoteStruct struct = new TrackedNoteStruct();
        public static class TrackedNoteStruct implements Struct<TrackedNote> {
            @Override
            public Class<TrackedNote> getTypeClass() {
                return TrackedNote.class;
            }

            @Override
            public String getTypeString() {
                return "struct:TrackedNote";
            }

            @Override
            public int getSize() {
                return kSizeDouble + Translation2d.struct.getSize();
            }

            @Override
            public String getSchema() {
                return "Translation2d fieldPos;double confidence";
            }

            @Override
            public Struct<?>[] getNested() {
                return new Struct<?>[] {Translation2d.struct};
            }

            @Override
            public TrackedNote unpack(ByteBuffer bb) {
                var fieldPos = Translation2d.struct.unpack(bb);
                var confidence = bb.getDouble();
                return new TrackedNote(fieldPos, confidence);
            }

            @Override
            public void pack(ByteBuffer bb, TrackedNote value) {
                Translation2d.struct.pack(bb, value.fieldPos);
                bb.putDouble(value.confidence);
            }
        }
    }
}
