package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.vision.apriltag.ApriltagCameraIO.ApriltagCameraResult;

public class ApriltagCamera {

    private final String name;
    private final ApriltagCameraIO cameraIO;
    private final ApriltagCameraIOInputsAutoLogged inputs = new ApriltagCameraIOInputsAutoLogged();

    public ApriltagCamera(String name, ApriltagCameraIO cameraIO) {
        this.name = name;
        this.cameraIO = cameraIO;
    }

    public Optional<ApriltagCameraResult> periodic() {
        cameraIO.updateInputs(inputs);
        Logger.processInputs("ApriltagVision/Camera/" + name, inputs);
        return inputs.getResult();
    }
}