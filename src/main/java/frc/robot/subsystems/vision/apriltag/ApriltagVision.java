package frc.robot.subsystems.vision.apriltag;

import frc.robot.util.VirtualSubsystem;

public class ApriltagVision extends VirtualSubsystem {

    private final ApriltagCamera[] cameras;

    public ApriltagVision(ApriltagCamera... cameras) {
        System.out.println("[Init ApriltagVision] Instantiating ApriltagVision");
        this.cameras = cameras;
    }

    @Override
    public void periodic() {
        for (var camera : cameras) {
            camera.periodic();
        }
    }
}