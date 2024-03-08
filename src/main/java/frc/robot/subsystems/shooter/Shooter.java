// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static final LoggedTunableNumber maxRPS = new LoggedTunableNumber("Shooter/Rotations Per Second", 65);
  private static final LoggedTunableNumber ampRPS = new LoggedTunableNumber("Shooter/Amp RPS", 30);

  private static final double smoothingFactor = 0.15;
  private double smoothedAverageRPS;

  private static final double followUpTime = 0.25;
  private final Timer followUpTimer = new Timer();

  private static final LoggedTunableNumber readyToShootTolerance = new LoggedTunableNumber("Shooter/Ready To Shoot Tolerance", 3);
  private boolean readyToShoot;

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

  public boolean readyToShoot() {
    return readyToShoot;
  }

  private double getAverageRPS() {
    return Units.radiansToRotations((inputs.leftMotor.velocityRadPerSec + inputs.rightMotor.velocityRadPerSec) * 0.5);
  }

  public boolean shot() {
    return getAverageRPS() < smoothedAverageRPS - 4;
  }

  public Command shootWith(DoubleSupplier rps) {
    return new FunctionalCommand(
      () -> {
        smoothedAverageRPS = getAverageRPS();
      },
      () -> {
        var speed = rps.getAsDouble();
        shooterIO.setLeftVelocity(speed);
        shooterIO.setRightVelocity(speed);
        readyToShoot = MathUtil.isNear(speed, getAverageRPS(), readyToShootTolerance.get());
        Logger.recordOutput("Shooter/Target RPS", speed);
      },
      (interrupted) -> {
        shooterIO.setLeftVelocity(0);
        shooterIO.setRightVelocity(0);
        readyToShoot = false;
        Logger.recordOutput("Shooter/Target RPS", 0);
      },
      () -> followUpTimer.hasElapsed(followUpTime),
      this
    );
  }

  public Command shootWithTunableNumber() {
    return shootWith(maxRPS::get).withName("Shoot with tunable number");
  }

  public Command shoot(Supplier<Translation2d> FORR) {
    return shootWith(() -> ShooterConstants.distanceLerp(FORR.get().getNorm(), ShooterConstants.RPS)).withName("Shoot at pos");
  }

  public Command preemptiveSpinup() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        shooterIO.setLeftVelocity(45);
        shooterIO.setRightVelocity(45);
        Logger.recordOutput("Shooter/Target RPS", 45);
      },
      (interrupted) -> {
        shooterIO.setLeftVelocity(0);
        shooterIO.setRightVelocity(0);
        Logger.recordOutput("Shooter/Target RPS", 0);
      },
      () -> false,
      this
    ).withName("Pre-emptive Spinup");
  }

  public Command amp() {
    var subsystem = this;
    return new Command() {
      {
        addRequirements(subsystem);
        setName("Amp");
      }
      @Override
      public void execute() {
        shooterIO.setLeftVelocity(ampRPS.get());
        shooterIO.setRightVelocity(ampRPS.get());
        Logger.recordOutput("Shooter/Target RPS", ampRPS.get());
      }
      @Override
      public void end(boolean interrupted) {
        shooterIO.setLeftVelocity(0);
        shooterIO.setRightVelocity(0);
        Logger.recordOutput("Shooter/Target RPS", 0);
      }
    };
  }
}
