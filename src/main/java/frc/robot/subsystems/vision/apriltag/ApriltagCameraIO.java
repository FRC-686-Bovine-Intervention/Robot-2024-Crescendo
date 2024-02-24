package frc.robot.subsystems.vision.apriltag;

import java.nio.ByteBuffer;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public interface ApriltagCameraIO {

    @AutoLog
    public class ApriltagCameraIOInputs {
        public boolean isConnected;
        public boolean hasResult;
        public ApriltagCameraResult result;

        public Optional<ApriltagCameraResult> getResult() {
            return Optional.ofNullable(hasResult ? result : null);
        }

        public Optional<Pose3d> getPose() {
            return getResult().map((r) -> r.estimatedRobotPose);
        }
    }

    public static class ApriltagCameraResult implements StructSerializable {
        public final double timestamp;
        public final double cameraToTargetDist;
        public final Pose3d estimatedRobotPose;

        public ApriltagCameraResult(double timestamp, double cameraToTargetDist, Pose3d estimatedRobotPose) {
            this.timestamp = timestamp;
            this.cameraToTargetDist = cameraToTargetDist;
            this.estimatedRobotPose = estimatedRobotPose;
        }

        public static ApriltagCameraResult from(PhotonPipelineResult result, EstimatedRobotPose estimatedRobotPose) {
            return new ApriltagCameraResult(estimatedRobotPose.timestampSeconds, result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm(), estimatedRobotPose.estimatedPose);
        }

        public static final ApriltagCameraResultStruct struct = new ApriltagCameraResultStruct();
        public static class ApriltagCameraResultStruct implements Struct<ApriltagCameraResult> {
            @Override
            public Class<ApriltagCameraResult> getTypeClass() {
                return ApriltagCameraResult.class;
            }

            @Override
            public String getTypeString() {
                return "struct:ApriltagCameraResult";
            }

            @Override
            public int getSize() {
                return kSizeDouble * 2 + Pose3d.struct.getSize();
            }

            @Override
            public String getSchema() {
                return "double timestamp;double cameraToTargetDist;Pose3d estimatedRobotPose";
            }

            @Override
            public Struct<?>[] getNested() {
                return new Struct<?>[]{Pose3d.struct};
            }

            @Override
            public ApriltagCameraResult unpack(ByteBuffer bb) {
                var timestamp = bb.getDouble();
                var cameraToTargetDist = bb.getDouble();
                var estimatedRobotPose = Pose3d.struct.unpack(bb);
                return new ApriltagCameraResult(timestamp, cameraToTargetDist, estimatedRobotPose);
            }

            @Override
            public void pack(ByteBuffer bb, ApriltagCameraResult value) {
                bb.putDouble(value.timestamp);
                bb.putDouble(value.cameraToTargetDist);
                Pose3d.struct.pack(bb, value.estimatedRobotPose);
            }
        }
    }
    
    public default void updateInputs(ApriltagCameraIOInputs inputs) {}
}
