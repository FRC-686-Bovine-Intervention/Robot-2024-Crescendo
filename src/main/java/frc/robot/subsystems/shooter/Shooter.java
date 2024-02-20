// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final LoggedTunableNumber maxRPS = new LoggedTunableNumber("Shooter/Max Rotations Per Second", 45);
  private final LoggedTunableNumber spinRPS = new LoggedTunableNumber("Shooter/Spin in Rotations Per Second", 0);

  private static final double smoothingFactor = 0.15;
  private double smoothedAverageRPS;

  private static final double followUpTime = 0.5;
  private final Timer followUpTimer = new Timer();

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
    SmartDashboard.putData("Subsystems/Shooter", this);
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    smoothedAverageRPS = (getAverageRPS() * smoothingFactor) + (smoothedAverageRPS * (1-smoothingFactor));
    if(getCurrentCommand() == null) {
      followUpTimer.stop();
      followUpTimer.reset();
    }
    if(shot()) {
      followUpTimer.start();
    }
    Logger.recordOutput("Shooter/Average RPS", getAverageRPS());
    Logger.recordOutput("Shooter/Smoothed RPS", smoothedAverageRPS);
    Logger.recordOutput("Shooter/Timer", followUpTimer.get());
    Logger.recordOutput("Shooter/Shot", shot());
  }

  private double getAverageRPS() {
    return Units.radiansToRotations((inputs.leftMotor.velocityRadPerSec + inputs.rightMotor.velocityRadPerSec) * 0.5);
  }

  private boolean shot() {
    return getAverageRPS() < smoothedAverageRPS - 4;
  }

  private Command shootWith(DoubleSupplier rps) {
    return new FunctionalCommand(
      () -> {},
      () -> {
        shooterIO.setLeftVelocity(rps.getAsDouble());
        shooterIO.setRightVelocity(rps.getAsDouble());
      },
      (interrupted) -> {
        shooterIO.setLeftVelocity(0);
        shooterIO.setRightVelocity(0);
      },
      () -> followUpTimer.hasElapsed(followUpTime),
      this
    );
  }

  public Command shootWithTunableNumber() {
    return shootWith(maxRPS::get);
  }

  public Command shoot() {
    return shootWith(() -> {
      int lowerBound = 0;
      int upperBound = 0;
      double distanceToSpeaker = FieldConstants.speakerCenter.getDistance(RobotState.getInstance().getPose().getTranslation());
      for(int i = 0; i < ShooterConstants.distance.length; i++) {
        upperBound = i;
        if(distanceToSpeaker < ShooterConstants.distance[i]) {
          break;
        }
        lowerBound = i;
      }
      double t = MathUtil.inverseInterpolate(ShooterConstants.distance[lowerBound], ShooterConstants.distance[upperBound], distanceToSpeaker);
      double rps = MathUtil.interpolate(ShooterConstants.RPS[lowerBound], ShooterConstants.RPS[upperBound], t);
      return rps;
    });
  }
}
