package frc.robot.subsystems.vision.note;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.note.NoteVision.TrackedNote;
import frc.robot.util.Alert;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Alert.AlertType;

public class NoteVisionIOPhotonVision implements NoteVisionIO {
    private final PhotonCamera cam;
    private final Camera camMeta;
    
    private static final LoggedTunableNumber targetPitchThreshold = new LoggedTunableNumber("Vision/Pitch Threshold", 0.0);

    public NoteVisionIOPhotonVision(Camera camera) {
        this.cam = new PhotonCamera(camera.hardwareName);
        this.camMeta = camera;

        notConnectedAlert = new Alert(camMeta.hardwareName + " is not connected", AlertType.ERROR);
    }

    private final Alert notConnectedAlert;
    @Override
    public void updateInputs(NoteVisionIOInputs inputs) {
        inputs.connected = cam.isConnected();
        notConnectedAlert.set(!inputs.connected);
        inputs.trackedNotes = new TrackedNote[0];
        if(!inputs.connected) return;
        inputs.trackedNotes = 
            cam
            .getLatestResult()
            .getTargets()
            .stream()
            // .filter((target) -> (camMeta.getRobotToCam().getRotation().getY() + target.getPitch()) < targetPitchThreshold.get())
            .map(this::resultToTargets)
            .toArray(TrackedNote[]::new);
    }

    private TrackedNote resultToTargets(PhotonTrackedTarget target) {
        var targetCamViewTransform = camMeta.getRobotToCam().plus(
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
        var camToTargetTranslation = new Translation3d(distOut, distOff, -camMeta.getRobotToCam().getZ());
        var fieldPos = new Pose3d(RobotState.getInstance().getPose())
            .transformBy(camMeta.getRobotToCam())
            .transformBy(new Transform3d(camToTargetTranslation, new Rotation3d()))
            .toPose2d()
            .getTranslation();

        var confidence = Math.sqrt(target.getArea()) * NoteVision.confidencePerAreaPercent.get();

        return new TrackedNote(fieldPos, confidence);
    }
}
