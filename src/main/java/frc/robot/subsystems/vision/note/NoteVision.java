package frc.robot.subsystems.vision.note;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.util.VirtualSubsystem;

public class NoteVision extends VirtualSubsystem {
    private final ArrayList<TrackedNote> trackedNotes = new ArrayList<>();

    private final PhotonCamera cam = new PhotonCamera("TestCam");

    private final Transform3d robotToCam = new Transform3d(new Translation3d(0, 0, 0.73), new Rotation3d());

    @Override
    public void periodic() {
        var photonTargets = cam.getLatestResult().targets;

        Logger.recordOutput("Vision/pitch", photonTargets.stream().map((target) -> new Translation2d(target.getPitch(), target.getYaw())).toArray(Translation2d[]::new));
        Logger.recordOutput("Vision/Best Cam to Targets", photonTargets.stream().map((target) -> robotToCam.plus(new Transform3d(new Translation3d(), new Rotation3d(0, Units.degreesToRadians(-target.getPitch()), Units.degreesToRadians(-target.getYaw()))))).toArray(Transform3d[]::new));
        Logger.recordOutput("Vision/Targets from Trig", photonTargets.stream().map(
            (target) -> robotToCam.plus(
                new Transform3d(
                    new Translation3d(),
                    new Rotation3d(
                        0,
                        Units.degreesToRadians(-target.getPitch()),
                        Units.degreesToRadians(-target.getYaw())
                    )
                )
            )
        ).map(
            (targetCamViewTransform) -> {
                var distOut = targetCamViewTransform.getTranslation().getZ() / Math.tan(targetCamViewTransform.getRotation().getY());
                var distOff = distOut * Math.tan(targetCamViewTransform.getRotation().getZ());
                return new Translation3d(distOut, distOff, -robotToCam.getZ());
            }
        ).map(
            (camToTargetTranslation) -> 
                new Pose3d(RobotState.getInstance().getPose()).transformBy(robotToCam).transformBy(new Transform3d(camToTargetTranslation, new Rotation3d()))
        )/* .map(
            (target3dPose) -> target3dPose.toPose2d().getTranslation()
        ) */.toArray(Pose3d[]::new));
        // for(var target : photonTargets) {
        // }
    }

    public static class TrackedNote {
        public Translation2d fieldPos;
        public double confidence;

        public TrackedNote(Translation2d fieldPos, double confidence) {
            this.fieldPos = fieldPos;
            this.confidence = confidence;
        }
    }
}
