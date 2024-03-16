// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
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
    private static final LoggedTunableNumber ampMPS = new LoggedTunableNumber("Shooter/Amp MPS", 3);
    private static final LoggedTunableNumber preemtiveMPS = new LoggedTunableNumber("Shooter/Pre-emptive MPS", 30);
    // private static final LoggedTunableNumber shotDetMPS = new LoggedTunableNumber("Shooter/Shot Detection/MPS", 1);
    // private static final LoggedTunableNumber shotDetCurrent = new LoggedTunableNumber("Shooter/Shot Detection/Current", 2);

    // private static final LoggedTunableNumber surfaceSpeedSmoothingFactor = new LoggedTunableNumber("Shooter/Smoothing/MPS", 0.15);
    // private double smoothedAverageSurfaceSpeed;
    // private static final LoggedTunableNumber currentSmoothingFactor = new LoggedTunableNumber("Shooter/Smoothing/Current", 0.3);
    // private double smoothedAverageCurrent;

    private static final LoggedTunableNumber followUpTime = new LoggedTunableNumber("Shooter/Follow Up Time", 0.25);
    private final Timer followUpTimer = new Timer();

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
        // smoothedAverageSurfaceSpeed = MathUtil.interpolate(smoothedAverageSurfaceSpeed, getAverageSurfaceSpeed(), surfaceSpeedSmoothingFactor.get());
        // smoothedAverageCurrent = MathUtil.interpolate(smoothedAverageCurrent, getAverageCurrent(), currentSmoothingFactor.get());
        // if(endingCommand()) {
        //     followUpTimer.stop();
        //     followUpTimer.reset();
        // }
        // if(shot()) {
        //     followUpTimer.start();
        // }
        Logger.recordOutput("Shooter/Average RPS", getAverageSurfaceSpeed());
        // Logger.recordOutput("Shooter/Smoothed RPS", smoothedAverageSurfaceSpeed);
        // Logger.recordOutput("Shooter/Average Current", getAverageCurrent());
        // Logger.recordOutput("Shooter/Smoothed Current", smoothedAverageCurrent);
        Logger.recordOutput("Shooter/Timer", followUpTimer.get());
        // Logger.recordOutput("Shooter/Shot", shot());
        // Logger.recordOutput("Shooter/Surface Correct", surfaceSpeedCorrect());
        // Logger.recordOutput("Shooter/Current Correct", currentCorrect());
    }

    public boolean readyToShoot() {
        return readyToShoot;
    }

    private double getAverageSurfaceSpeed() {
        return ShooterConstants.motorToSurface.radsToSurface(MathExtraUtil.average(inputs.leftMotor.velocityRadPerSec, inputs.rightMotor.velocityRadPerSec));
    }

    // private double getAverageCurrent() {
    //     return MathExtraUtil.average(inputs.leftMotor.currentAmps, inputs.rightMotor.currentAmps);
    // }

    // private boolean surfaceSpeedCorrect() {
    //     return getAverageSurfaceSpeed() < smoothedAverageSurfaceSpeed - shotDetMPS.get();
    // }
    // private boolean currentCorrect() {
    //     return getAverageCurrent() > smoothedAverageCurrent + shotDetCurrent.get();
    // }
    // public boolean shot() {
    //     return surfaceSpeedCorrect() && currentCorrect();
    // }
    // public boolean endingCommand() {
    //     return followUpTimer.hasElapsed(followUpTime.get());
    // }

    private Command surfaceSpeed(DoubleSupplier mps) {
        return surfaceSpeed(mps, () -> mps.getAsDouble() - 1);
    }

    private Command surfaceSpeed(DoubleSupplier mps, DoubleSupplier acceptableMPS) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Set Surface Speed");
            }
            @Override
            public void initialize() {
                execute();
            }
            @Override
            public void execute() {
                var speed = mps.getAsDouble();
                shooterIO.setLeftSurfaceSpeed(speed);
                shooterIO.setRightSurfaceSpeed(speed);
                readyToShoot = getAverageSurfaceSpeed() >= acceptableMPS.getAsDouble() && getAverageSurfaceSpeed() <= mps.getAsDouble() + 2;
            }
            @Override
            public void end(boolean interrupted) {
                followUpTimer.stop();
                followUpTimer.reset();
                shooterIO.stop();
                readyToShoot = false;
            }
        };
    }

    private Command followUp(BooleanSupplier shot) {
        return new Command() {
            {
                setName("Wait for Followup");
            }
            @Override
            public void execute() {
                if(shot.getAsBoolean()) {
                    followUpTimer.start();
                }
            }
            @Override
            public void end(boolean interrupted) {
                followUpTimer.stop();
                followUpTimer.reset();
            }
            @Override
            public boolean isFinished() {
                return followUpTimer.hasElapsed(followUpTime.get());
            }
        };
    }

    private Command surfaceSpeedWithFinish(DoubleSupplier mps, BooleanSupplier shot) {
        return surfaceSpeedWithFinish(mps, () -> mps.getAsDouble() - 1, shot);
    }
    private Command surfaceSpeedWithFinish(DoubleSupplier mps, DoubleSupplier acceptableMPS, BooleanSupplier shot) {
        return followUp(shot).deadlineWith(surfaceSpeed(mps, acceptableMPS)).withName("Set Surface Speed Finish");
    }

    public Command shootWithTunableNumber() {
        return surfaceSpeed(tuningMPS::get).withName("Shoot with tunable number");
    }

    public Command shoot(Supplier<Translation2d> FORR, BooleanSupplier shot) {
        return surfaceSpeedWithFinish(() -> ShooterConstants.distLerp(FORR.get().getNorm(), ShooterConstants.surfaceSpeed), () -> ShooterConstants.distLerp(FORR.get().getNorm(), ShooterConstants.acceptableSurfaceSpeed), shot).withName("Shoot at pos");
    }

    public Command preemptiveSpinup() {
        return surfaceSpeed(preemtiveMPS::get).withName("Pre-emptive Spinup");
    }

    public Command amp() {
        return surfaceSpeed(ampMPS::get, () -> 500).withName("Amp");
    }
}
