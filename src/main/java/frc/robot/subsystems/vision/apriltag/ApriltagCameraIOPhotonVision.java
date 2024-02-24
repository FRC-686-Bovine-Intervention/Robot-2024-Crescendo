package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.RobotState;

public class ApriltagCameraIOPhotonVision implements ApriltagCameraIO {

    private final PhotonCamera photonCam;
    private final Camera cam;
    private PhotonPoseEstimator photonPoseEstimator;

    public ApriltagCameraIOPhotonVision(Camera cam) {
        this.cam = cam;
        photonCam = new PhotonCamera(cam.hardwareName);

        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
            // String resourceFile = Filesystem.getDeployDirectory() + "/Bunnybots_2023.json";
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

            // test that I made the JSON file correctly
            // List<AprilTag> tags = fieldLayout.getTags();
            // for (AprilTag tag : tags) {
            //     System.out.println(tag);
            //     System.out.println(tag.pose.getRotation().toRotation2d());
            // }

            photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCam, cam.getRobotToCam());
            photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        } catch (Exception e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }
    }

    public void updateInputs(ApriltagCameraIOInputs inputs) {
        // set default values
        inputs.isConnected = photonCam.isConnected();
        inputs.visionPose = Optional.empty();
        inputs.timestamp = Timer.getFPGATimestamp();

        if ((!inputs.isConnected) || (photonPoseEstimator == null)) {
            return;
        }
        photonPoseEstimator.setRobotToCameraTransform(cam.getRobotToCam());

        // System.out.println("[DEBUG ApTagCamIOPhoton] Pre-set reference pose");
        photonPoseEstimator.setReferencePose(RobotState.getInstance().getPose());
        // System.out.println("[DEBUG ApTagCamIOPhoton] Post-set reference pose");
        // System.out.println("[DEBUG ApTagCamIOPhoton] Pre-estimator update");
        var result = photonCam.getLatestResult();
        Logger.recordOutput("photonresult", result);
        var optRobotPose = photonPoseEstimator.update(result);
        // System.out.println("[DEBUG ApTagCamIOPhoton] Post-estimator update");
        
        if (optRobotPose.isPresent()) {
            // System.out.println("[DEBUG ApTagCamIOPhoton] Camera has pose");
            inputs.visionPose = Optional.of(optRobotPose.get().estimatedPose);
            inputs.timestamp = optRobotPose.get().timestampSeconds;
        }
    }
}
