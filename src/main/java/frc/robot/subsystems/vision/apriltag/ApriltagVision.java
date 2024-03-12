package frc.robot.subsystems.vision.apriltag;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIO.ApriltagCameraResult;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;

public class ApriltagVision extends VirtualSubsystem {

    private final ApriltagCamera[] cameras;

    public ApriltagVision(ApriltagCamera... cameras) {
        System.out.println("[Init ApriltagVision] Instantiating ApriltagVision");
        this.cameras = cameras;
    }

    private static final LoggedTunableNumber rejectDist = new LoggedTunableNumber("Vision/Apriltags/Reject Distance", 4);

    @Override
    public void periodic() {
        var results = Arrays.stream(cameras).map((c) -> c.periodic()).filter((o) -> o.isPresent()).map((r) -> r.get()).toArray(ApriltagCameraResult[]::new);
        var accepted = Arrays.stream(results).filter((r) -> r.cameraToTargetDist < rejectDist.get()).toArray(ApriltagCameraResult[]::new);
        var rejected = Arrays.stream(results).filter((r) -> r.cameraToTargetDist >= rejectDist.get()).toArray(ApriltagCameraResult[]::new);
        Logger.recordOutput("Vision/Apriltags/Accepted Poses", Arrays.stream(accepted).map((r) -> r.estimatedRobotPose).toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Apriltags/Rejected Poses", Arrays.stream(rejected).map((r) -> r.estimatedRobotPose).toArray(Pose3d[]::new));
        Arrays.stream(accepted).forEach((r) -> 
            RobotState.getInstance().addVisionMeasurement(
                r.estimatedRobotPose.toPose2d(),
                computeStdDevs(r.cameraToTargetDist),
                r.timestamp
            )
        );
    }

    private static final LoggedTunableNumber kTransA = new LoggedTunableNumber("Vision/Apriltags/StdDevs/Translational/aCoef", 1);
    private static final LoggedTunableNumber kTransC = new LoggedTunableNumber("Vision/Apriltags/StdDevs/Translational/cCoef", 0.75);
    private static final LoggedTunableNumber kRotA = new LoggedTunableNumber("Vision/Apriltags/StdDevs/Rotational/aCoef", 5);
    private static final LoggedTunableNumber kRotC = new LoggedTunableNumber("Vision/Apriltags/StdDevs/Rotational/cCoef", 1000);
    private static final LoggedTunableNumber kRotCDisabled = new LoggedTunableNumber("Vision/Apriltags/StdDevs/Rotational/disabledcCoef", 5);

    private Matrix<N3, N1> computeStdDevs(double distance) {
        double transStdDev = kTransA.get() * distance * distance + kTransC.get();
        double rotStdDev = kRotA.get() * distance * distance + (DriverStation.isEnabled() ? kRotC.get() : kRotCDisabled.get());
        return VecBuilder.fill(transStdDev, transStdDev, rotStdDev);
    }
}