// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MathExtraUtil;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private static final LoggedTunableNumber tuningMPS = new LoggedTunableNumber("Shooter/Tuning MPS", 30);
    private static final LoggedTunableNumber ampMPS = new LoggedTunableNumber("Shooter/Amp MPS", 20);
    private static final LoggedTunableNumber preemtiveMPS = new LoggedTunableNumber("Shooter/Pre-emptive MPS", 30);
    private static final LoggedTunableNumber shotDetMPS = new LoggedTunableNumber("Shooter/Shot Detection MPS", 1);

    private static final double smoothingFactor = 0.15;
    private double smoothedAverageSurfaceSpeed;

    private static final LoggedTunableNumber followUpTime = new LoggedTunableNumber("Shooter/Follow Up Time", 0.5);
    private final Timer followUpTimer = new Timer();

    // private static final LoggedTunableNumber readyToShootTolerance = new LoggedTunableNumber("Shooter/Ready To Shoot Tolerance", 1.5);
    private boolean readyToShoot;

    public Shooter(ShooterIO shooterIO) {
        System.out.println("[Init Shooter] Instantiating Pivot");
        this.shooterIO = shooterIO;
        System.out.println("[Init Shooter] Shooter IO: " + this.shooterIO.getClass().getSimpleName());
        SmartDashboard.putData("Subsystems/Shooter", this);
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        smoothedAverageSurfaceSpeed = (getAverageSurfaceSpeed() * smoothingFactor) + (smoothedAverageSurfaceSpeed * (1-smoothingFactor));
        if(getCurrentCommand() == null) {
            followUpTimer.stop();
            followUpTimer.reset();
        }
        if(shot()) {
            followUpTimer.start();
        }
        Logger.recordOutput("Shooter/Average RPS", getAverageSurfaceSpeed());
        Logger.recordOutput("Shooter/Smoothed RPS", smoothedAverageSurfaceSpeed);
        Logger.recordOutput("Shooter/Timer", followUpTimer.get());
        Logger.recordOutput("Shooter/Shot", shot());
    }

    public boolean readyToShoot() {
        return readyToShoot;
    }

    private double getAverageSurfaceSpeed() {
        return ShooterConstants.motorToSurface.apply(MathExtraUtil.average(inputs.leftMotor.velocityRadPerSec, inputs.rightMotor.velocityRadPerSec));
    }

    public boolean shot() {
        return getAverageSurfaceSpeed() < smoothedAverageSurfaceSpeed - shotDetMPS.get();
    }

    private Command surfaceSpeed(DoubleSupplier MPS) {
        return surfaceSpeed(MPS, () -> MPS.getAsDouble() - 1);
    }

    private Command surfaceSpeed(DoubleSupplier MPS, DoubleSupplier acceptableMPS) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Set Surface Speed");
            }
            @Override
            public void initialize() {
                smoothedAverageSurfaceSpeed = getAverageSurfaceSpeed();
            }
            @Override
            public void execute() {
                var speed = MPS.getAsDouble();
                shooterIO.setLeftSurfaceSpeed(speed);
                shooterIO.setRightSurfaceSpeed(speed);
                readyToShoot = getAverageSurfaceSpeed() >= acceptableMPS.getAsDouble();
            }
            @Override
            public void end(boolean interrupted) {
                shooterIO.stop();
                readyToShoot = false;
            }
        };
    }

    private Command surfaceSpeedWithFinish(DoubleSupplier mps) {
        return surfaceSpeed(mps).until(() -> followUpTimer.hasElapsed(followUpTime.get())).withName("Set Surface Speed Finish");
    }
    private Command surfaceSpeedWithFinish(DoubleSupplier mps, DoubleSupplier acceptableMPS) {
        return surfaceSpeed(mps, acceptableMPS).until(() -> followUpTimer.hasElapsed(followUpTime.get())).withName("Set Surface Speed Finish");
    }

    public Command shootWithTunableNumber() {
        return surfaceSpeed(tuningMPS::get).withName("Shoot with tunable number");
    }

    public Command shoot(Supplier<Translation2d> FORR) {
        return surfaceSpeedWithFinish(() -> ShooterConstants.distLerp(FORR.get().getNorm(), ShooterConstants.surfaceSpeed), () -> ShooterConstants.distLerp(FORR.get().getNorm(), ShooterConstants.acceptableSurfaceSpeed)).withName("Shoot at pos");
    }

    public Command preemptiveSpinup() {
        return surfaceSpeed(preemtiveMPS::get).withName("Pre-emptive Spinup");
    }

    public Command amp() {
        return surfaceSpeedWithFinish(ampMPS::get).withName("Amp");
    }
}
