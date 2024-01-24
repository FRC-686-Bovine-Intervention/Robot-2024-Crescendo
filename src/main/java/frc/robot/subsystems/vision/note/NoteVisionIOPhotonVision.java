package frc.robot.subsystems.vision.note;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.note.NoteVision.TrackedNote;

public class NoteVisionIOPhotonVision implements NoteVisionIO {
    private static final PhotonCamera cam = new PhotonCamera("TestCam");
    private static final Transform3d robotToCam = new Transform3d(new Translation3d(0, 0, 0.73), new Rotation3d());

    @Override
    public void updateInputs(NoteVisionIOInputs inputs) {
        inputs.trackedNotes = 
            cam
            .getLatestResult()
            .targets
            .stream()
            .map(NoteVisionIOPhotonVision::resultToTargets)
            .toArray(TrackedNote[]::new);
    }

    private static TrackedNote resultToTargets(PhotonTrackedTarget target) {
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
        var fieldPos = new Pose3d(RobotState.getInstance().getPose())
            .transformBy(robotToCam)
            .transformBy(new Transform3d(camToTargetTranslation, new Rotation3d()))
            .toPose2d()
            .getTranslation();

        var confidence = target.getArea() * NoteVision.confidencePerAreaPercent.get();

        return new TrackedNote(fieldPos, confidence);
    }
}
