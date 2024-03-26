// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Queue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(Constants.CANDevices.pigeonCanID, Constants.CANDevices.driveCanBusName);

  private final StatusSignal<Double> yaw;
  private final Queue<Double> yawPositionQueue;

  public GyroIOPigeon2() {
    var config = new Pigeon2Configuration();
    // change factory defaults here
    config.MountPose.MountPoseYaw = 91.9501;    // pigeon2 oriented with x forward, y left, z up
    config.MountPose.MountPosePitch = 0.47715;
    config.MountPose.MountPoseRoll = -0.679168;
    pigeon.getConfigurator().apply(config);

    yaw = pigeon.getYaw();

    // set signals to an appropriate rate
    yaw.setUpdateFrequency(DriveConstants.odometryFrequency);
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon, yaw);

    pigeon.setYaw(0);
  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = pigeon.getYaw().getStatus().isOK();

    inputs.yawPositionRad =     Units.degreesToRadians( pigeon.getYaw().getValue());    // ccw+
    inputs.pitchPositionRad =   Units.degreesToRadians(-pigeon.getPitch().getValue());  // up+
    inputs.rollPositionRad =    Units.degreesToRadians(-pigeon.getRoll().getValue());   // ccw+

    inputs.yawVelocityRadPerSec =   Units.degreesToRadians( pigeon.getAngularVelocityZWorld().getValue());   // ccw+
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-pigeon.getAngularVelocityYWorld().getValue());   // up+
    inputs.rollVelocityRadPerSec =  Units.degreesToRadians(-pigeon.getAngularVelocityXWorld().getValue());   // ccw+

    inputs.odometryYawPositions = yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    yawPositionQueue.clear();
  }
}
