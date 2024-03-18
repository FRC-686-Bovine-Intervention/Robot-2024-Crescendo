package frc.robot.subsystems.vision.apriltag;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.RobotState;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class ApriltagCameraIOPhotonVision implements ApriltagCameraIO {

    private final PhotonCamera photonCam;
    private final Camera cam;
    private PhotonPoseEstimator photonPoseEstimator;

    public ApriltagCameraIOPhotonVision(Camera cam) {
        this.cam = cam;
        photonCam = new PhotonCamera(cam.hardwareName);

        var fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCam, cam.getRobotToCam());
        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        notConnectedAlert = new Alert(cam.name() + " is not connected", AlertType.ERROR);
    }

    private final Alert notConnectedAlert;
    public void updateInputs(ApriltagCameraIOInputs inputs) {
        // set default values
        inputs.isConnected = photonCam.isConnected();
        notConnectedAlert.set(!inputs.isConnected);
        inputs.hasResult = false;

        if ((!inputs.isConnected) || (photonPoseEstimator == null)) return;

        photonPoseEstimator.setRobotToCameraTransform(cam.getRobotToCam());
        photonPoseEstimator.setReferencePose(RobotState.getInstance().getPose());

        var result = photonCam.getLatestResult();
        Logger.recordOutput("Vision/Apriltags/Result", result);
        var optRobotPose = photonPoseEstimator.update(result);
        
        optRobotPose.ifPresent((e) -> {
            inputs.hasResult = true;
            inputs.result = ApriltagCameraResult.from(result, e);
        });
    }
}
