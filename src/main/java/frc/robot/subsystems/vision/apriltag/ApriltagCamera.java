package frc.robot.subsystems.vision.apriltag;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

public class ApriltagCamera {

    private final String name;
    private final ApriltagCameraIO cameraIO;
    private final ApriltagCameraIOInputsAutoLogged inputs = new ApriltagCameraIOInputsAutoLogged();

    public ApriltagCamera(String name, ApriltagCameraIO cameraIO) {
        this.name = name;
        this.cameraIO = cameraIO;
    }

    public void periodic() {
        cameraIO.updateInputs(inputs);
        Logger.processInputs("Vision/Camera/" + name, inputs);
        
        if(inputs.hasResult && inputs.result.cameraToTargetDist < 3) {
            RobotState.getInstance().addVisionMeasurement(
                inputs.result.estimatedRobotPose.toPose2d(),
                computeStdDevs(inputs.result.cameraToTargetDist),  // TODO: figure out vision stdDevs 
                inputs.result.timestamp
            );
        }
    }

    private static final LoggedTunableNumber kTransA = new LoggedTunableNumber("Vision/StdDevs/Translational/aCoef", 1);
    private static final LoggedTunableNumber kTransC = new LoggedTunableNumber("Vision/StdDevs/Translational/cCoef", 0.75);
    private static final LoggedTunableNumber kRotA = new LoggedTunableNumber("Vision/StdDevs/Rotational/aCoef", 5);
    private static final LoggedTunableNumber kRotC = new LoggedTunableNumber("Vision/StdDevs/Rotational/cCoef", 1000);
    private static final LoggedTunableNumber kRotCDisabled = new LoggedTunableNumber("Vision/StdDevs/Rotational/disabledcCoef", 5);

    private Matrix<N3, N1> computeStdDevs(double distance) {
        double transStdDev = kTransA.get() * distance * distance + kTransC.get();
        double rotStdDev = kRotA.get() * distance * distance + (DriverStation.isEnabled() ? kRotC.get() : kRotCDisabled.get());
        return VecBuilder.fill(transStdDev, transStdDev, rotStdDev);
    }
}