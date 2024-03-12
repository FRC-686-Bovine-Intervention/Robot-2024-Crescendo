// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MathExtraUtil;
import frc.robot.util.TunableFeedForward;
import frc.robot.util.TunableProfiledPID;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private static final LoggedTunableNumber tuningMPS = new LoggedTunableNumber("Shooter/Tuning MPS", 65);
    private static final LoggedTunableNumber ampMPS = new LoggedTunableNumber("Shooter/Amp MPS", 30);
    private static final LoggedTunableNumber preMPS = new LoggedTunableNumber("Shooter/Preemptive MPS", 30);
    private static final LoggedTunableNumber shotDetMPS = new LoggedTunableNumber("Shooter/Shot Detection", 1);

    private static final double smoothingFactor = 0.15;
    private double smoothedAverageSurfaceSpeed;

    private static final double followUpTime = 0.5;
    private final Timer followUpTimer = new Timer();

    private static final LoggedTunableNumber readyToShootTolerance = new LoggedTunableNumber("Shooter/Ready To Shoot Tolerance", 1.5);
    private boolean readyToShoot;

    private final ProfiledPIDController leftPID = new ProfiledPIDController(
        0,
        0,
        0,
        new Constraints(
            0,
            0
        )
    );
    private final ProfiledPIDController rightPID = new ProfiledPIDController(
        0,
        0,
        0,
        new Constraints(
            0,
            0
        )
    );
    private final TunableProfiledPID tunableLeftPID = new TunableProfiledPID("Shooter/PID", leftPID);
    private final TunableProfiledPID tunableRightPID = new TunableProfiledPID("Shooter/PID", rightPID);
    private final TunableFeedForward feedForward = new TunableFeedForward("Shooter/FF",
        0,
        0,
        0
    );

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
        tunableLeftPID.update();
        tunableRightPID.update();
        feedForward.update();
        smoothedAverageSurfaceSpeed = (getAverageSurfaceSpeed() * smoothingFactor) + (smoothedAverageSurfaceSpeed * (1-smoothingFactor));
        if(getCurrentCommand() == null) {
            followUpTimer.stop();
            followUpTimer.reset();
        }
        if(shot()) {
            followUpTimer.start();
        }
        Logger.recordOutput("Shooter/Average MPS", getAverageSurfaceSpeed());
        Logger.recordOutput("Shooter/Smoothed MPS", smoothedAverageSurfaceSpeed);
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

    private Command surfaceSpeed(DoubleSupplier surfaceSpeed) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Set Linear Speed");
            }
            @Override
            public void initialize() {
                smoothedAverageSurfaceSpeed = getAverageSurfaceSpeed();
            }
            @Override
            public void execute() {
                var goal = surfaceSpeed.getAsDouble();
                var leftPIDOut = leftPID.calculate(ShooterConstants.motorToSurface.apply(inputs.leftMotor.velocityRadPerSec), goal);
                var rightPIDOut = rightPID.calculate(ShooterConstants.motorToSurface.apply(inputs.rightMotor.velocityRadPerSec), goal);
                Logger.recordOutput("Shooter/Left Wheel/PID Out", leftPIDOut);
                Logger.recordOutput("Shooter/Left Wheel/Profile Position", leftPID.getSetpoint().position);
                Logger.recordOutput("Shooter/Left Wheel/Profile Velocity", leftPID.getSetpoint().velocity);
                Logger.recordOutput("Shooter/Right Wheel/PID Out", rightPIDOut);
                Logger.recordOutput("Shooter/Right Wheel/Profile Position", rightPID.getSetpoint().position);
                Logger.recordOutput("Shooter/Right Wheel/Profile Velocity", rightPID.getSetpoint().velocity);
                var leftFF = feedForward.ff.calculate(leftPID.getSetpoint().position, leftPID.getSetpoint().velocity);
                var rightFF = feedForward.ff.calculate(rightPID.getSetpoint().position, rightPID.getSetpoint().velocity);
                Logger.recordOutput("Shooter/Left Wheel/FF Out", leftFF);
                Logger.recordOutput("Shooter/Right Wheel/FF Out", rightFF);
                shooterIO.setLeftVoltage(leftPIDOut + leftFF);
                shooterIO.setRightVoltage(rightPIDOut + rightFF);
            }
            @Override
            public void end(boolean interrupted) {
                shooterIO.setLeftVoltage(0);
                shooterIO.setRightVoltage(0);
            }
        };
    }

    private Command surfaceSpeedWithFinish(DoubleSupplier surfaceSpeed) {
        return surfaceSpeed(surfaceSpeed).until(() -> followUpTimer.hasElapsed(followUpTime));
    }

    public Command shootWithTunableNumber() {
        return surfaceSpeed(tuningMPS::get).withName("Shoot with tunable number");
    }

    public Command shoot(Supplier<Translation2d> FORR) {
        return surfaceSpeedWithFinish(() -> ShooterConstants.distLerp(FORR.get().getNorm(), ShooterConstants.RPS)).withName("Shoot at pos");
    }

    public Command preemptiveSpinup() {
        return surfaceSpeed(preMPS::get).withName("Pre-emptive Spinup");
    }

    public Command amp() {
        return surfaceSpeedWithFinish(ampMPS::get).withName("Amp");
    }
}
