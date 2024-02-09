package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public interface ApriltagCameraIO {

    public class ApriltagCameraIOInputs implements LoggableInputs {
        public boolean isConnected;
        public Optional<Pose3d> visionPose = Optional.empty();
        public double cameraToTargetDist;
        public double timestamp;

        private static final Pose3d nanPose = 
            new Pose3d(
                new Translation3d(
                    Double.NaN,
                    Double.NaN,
                    Double.NaN
                ),
                new Rotation3d(
                    new Quaternion(
                        Double.NaN,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN
                    )
                )
            );
    
        // AdvantageKit's @AutoLog annotation and processInputs() function 
        // cannot handle Optional types,so we will manually replace
        // Optional.empty() <--> NaNs with our own toLog() and fromLog() functions

        @Override
        public final void toLog(LogTable table) {
            table.put("isConnected", isConnected);
            table.put("visionPose", visionPose.orElse(nanPose));
            table.put("timestamp", timestamp);
            table.put("camToTargetDist", cameraToTargetDist);
        }
    
        @Override
        public final void fromLog(LogTable table) {
            isConnected = table.get("isConnected", false);
            var data = table.get("visionPose", nanPose);
            timestamp = table.get("timestamp", 0.0);
            cameraToTargetDist = table.get("camToTargetDist", 0.0);
    
            if (Double.isNaN(data.getX())) {
                visionPose = Optional.empty();
            } else {
                visionPose = Optional.of(data);
            }
        }    
    }
    
    public default void updateInputs(ApriltagCameraIOInputs inputs) {}
}
