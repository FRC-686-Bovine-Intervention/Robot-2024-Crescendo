package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Transform3d;

public class ApriltagCameraIOCustomPhoton implements ApriltagCameraIO {
    
    // private final PhotonCamera camera;
    // private final Transform3d cameraToRobot;
    // private AprilTagFieldLayout fieldLayout;

    public ApriltagCameraIOCustomPhoton(String cameraName, Transform3d robotToCamera) {
        // camera = new PhotonCamera(cameraName);
        // cameraToRobot = robotToCamera.inverse();

        // try {
        //     // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
        //     String resourceFile = Filesystem.getDeployDirectory() + "/Bunnybots_2023.json";
        //     fieldLayout = new AprilTagFieldLayout(resourceFile);
        // } catch (IOException e) {
        //     DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        // }
    }

    public void updateInputs(ApriltagCameraIOInputs inputs) {
        // set default values
        // inputs.isConnected = camera.isConnected();
        // inputs.visionPose = Optional.empty();
        // inputs.timestamp = Timer.getFPGATimestamp();

        // if (!inputs.isConnected) {
        //     return;
        // }

        // Optional<Pose3d> optRobotPose = Optional.empty();
        // var latestResult = camera.getLatestResult();
        // if(latestResult.hasTargets()) {
        //     var bestTarg = latestResult.getBestTarget();
        //     var optTag = fieldLayout.getTagPose(bestTarg.getFiducialId());
        //     if(optTag.isPresent()) {
        //         optRobotPose = Optional.of(optTag.get().transformBy(bestTarg.getBestCameraToTarget().inverse()).transformBy(cameraToRobot));
        //     }
        // }
        
        // if (optRobotPose.isPresent()) {
        //     inputs.visionPose = optRobotPose;
        //     inputs.timestamp = latestResult.getTimestampSeconds();
        // }
    }    
}
