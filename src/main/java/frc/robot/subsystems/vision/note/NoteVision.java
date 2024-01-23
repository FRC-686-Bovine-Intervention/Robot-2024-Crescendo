package frc.robot.subsystems.vision.note;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;

public class NoteVision extends VirtualSubsystem {
    private final ArrayList<TrackedNote> noteMemories = new ArrayList<>();

    private static final PhotonCamera cam = new PhotonCamera("TestCam");
    private static final Transform3d robotToCam = new Transform3d(new Translation3d(0, 0, 0.73), new Rotation3d());

    private static final LoggedTunableNumber updateDistanceThreshold = new LoggedTunableNumber("Vision/updateDistanceThreshold", 5);
    private static final LoggedTunableNumber posUpdatingFilteringFactor = new LoggedTunableNumber("Vision/posUpdatingFilteringFactor", 0.8);
    private static final LoggedTunableNumber confidencePerAreaPercent = new LoggedTunableNumber("Vision/confidencePerAreaPercent", 1);
    private static final LoggedTunableNumber confidenceDecayPerSecond = new LoggedTunableNumber("Vision/confidenceDecayPerSecond", 1);

    @Override
    public void periodic() {
        var photonFrameTargets = 
            cam
            .getLatestResult()
            .targets
            .stream()
            .map(TrackedNote::new)
            .toList();
        var connections = new ArrayList<PhotonMemoryConnection>();
        noteMemories.forEach(
            (memory) -> photonFrameTargets.forEach(
                (target) -> {
                    if(memory.fieldPos.getDistance(target.fieldPos) < updateDistanceThreshold.get()) {
                        connections.add(new PhotonMemoryConnection(memory, target));
                    }
                }
            )
        );
        connections.sort((a, b) -> (int) Math.signum(a.getDistance() - b.getDistance()));
        var unusedMemories = new ArrayList<>(noteMemories);
        var unusedTargets = new ArrayList<>(photonFrameTargets);
        while(!connections.isEmpty()) {
            var confirmedConnection = connections.get(0);
            confirmedConnection.memory.updatePosWithFiltering(confirmedConnection.photonFrameTarget);
            unusedMemories.remove(confirmedConnection.memory);
            unusedTargets.remove(confirmedConnection.photonFrameTarget);
            connections.removeIf((connection) -> 
                connection.memory == confirmedConnection.memory
                ||
                connection.photonFrameTarget == confirmedConnection.photonFrameTarget
            );
        }
        unusedMemories.forEach((memory) -> memory.decayConfidence());
        unusedTargets.forEach((target) -> noteMemories.add(target));
        noteMemories.removeIf((memory) -> memory.confidence <= 0);

        Logger.recordOutput("Vision/Photon Frame Targets", photonFrameTargets.stream().map(NoteVision::targetToPose).toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Note Memories", noteMemories.stream().map(NoteVision::targetToPose).toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Note Confidence", noteMemories.stream().mapToDouble((note) -> note.confidence).toArray());
    }

    private static Pose3d targetToPose(TrackedNote note) {
        return new Pose3d(new Translation3d(note.fieldPos.getX(), note.fieldPos.getY(), 0), new Rotation3d(0, Units.degreesToRadians(-90), 0));
    }

    private static record PhotonMemoryConnection(TrackedNote memory, TrackedNote photonFrameTarget) {
        public double getDistance() {
            return memory.fieldPos.getDistance(photonFrameTarget.fieldPos);
        }
    }

    public static class TrackedNote {
        public Translation2d fieldPos;
        public double confidence;

        public TrackedNote(PhotonTrackedTarget target) {
            var targetCamViewTransform = robotToCam.plus(
                new Transform3d(
                    new Translation3d(),
                    new Rotation3d(
                        0,
                        Units.degreesToRadians(-target.getPitch()),
                        Units.degreesToRadians(-target.getYaw())
                    )
                )
            );
            var distOut = targetCamViewTransform.getTranslation().getZ() / Math.tan(targetCamViewTransform.getRotation().getY());
            var distOff = distOut * Math.tan(targetCamViewTransform.getRotation().getZ());
            var camToTargetTranslation = new Translation3d(distOut, distOff, -robotToCam.getZ());
            this.fieldPos = new Pose3d(RobotState.getInstance().getPose())
                .transformBy(robotToCam)
                .transformBy(new Transform3d(camToTargetTranslation, new Rotation3d()))
                .toPose2d()
                .getTranslation();

            this.confidence = target.getArea() * confidencePerAreaPercent.get();
        }

        public void updatePosWithFiltering(TrackedNote newNote) {
            this.fieldPos = fieldPos.interpolate(newNote.fieldPos, posUpdatingFilteringFactor.get());
            this.confidence = newNote.confidence;
        }

        public void decayConfidence() {
            this.confidence -= confidenceDecayPerSecond.get() * Constants.dtSeconds;
        }
    }
}
