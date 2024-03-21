// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;

public class ShooterIOFalcon implements ShooterIO {
    private final TalonFX leftMotor = new TalonFX(CANDevices.shooterLeftID);
    private final TalonFX rightMotor = new TalonFX(CANDevices.shooterRightID);

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/PID/kP", 0.005);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/PID/kI", 0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/PID/kD", 0);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/PID/Profile/kA", 120);
    private final LoggedTunableNumber kJ = new LoggedTunableNumber("Shooter/PID/Profile/kJ", 120);
    private final LoggedTunableNumber ffkV = new LoggedTunableNumber("Shooter/FF/kV", 0.4);
    private final LoggedTunableNumber ffkA = new LoggedTunableNumber("Shooter/FF/kA", 0);
    private final LoggedTunableNumber ffkG = new LoggedTunableNumber("Shooter/FF/kG", 0);
    private final LoggedTunableNumber ffkS = new LoggedTunableNumber("Shooter/FF/kS", 0);

    public ShooterIOFalcon() {
        var config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = ShooterConstants.motorToSurface.rotPerSurface();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotor.getConfigurator().apply(config);
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightMotor.getConfigurator().apply(config);
        updateTunables();

        leftMotor.getRotorVelocity().setUpdateFrequency(50);
        leftMotor.getClosedLoopOutput().setUpdateFrequency(50);
        leftMotor.getClosedLoopReference().setUpdateFrequency(50);
        rightMotor.getRotorVelocity().setUpdateFrequency(50);
        leftMotor.getStatorCurrent().setUpdateFrequency(50);
        rightMotor.getStatorCurrent().setUpdateFrequency(50);
        leftMotor.getClosedLoopReferenceSlope().setUpdateFrequency(50);
    }

    private void updateTunables() {
        if(
            kP.hasChanged(hashCode()) |
            kI.hasChanged(hashCode()) |
            kD.hasChanged(hashCode()) |
            kA.hasChanged(hashCode()) |
            kJ.hasChanged(hashCode()) |
            ffkV.hasChanged(hashCode()) |
            ffkA.hasChanged(hashCode()) |
            ffkG.hasChanged(hashCode()) |
            ffkS.hasChanged(hashCode())
        ) {
            var pidConfig = new Slot0Configs();
            var profileConfig = new MotionMagicConfigs();
            pidConfig.kP = kP.get();
            pidConfig.kI = kI.get();
            pidConfig.kD = kD.get();
            profileConfig.MotionMagicAcceleration = kA.get();
            profileConfig.MotionMagicJerk = kJ.get();
            pidConfig.kV = ffkV.get();
            pidConfig.kA = ffkA.get();
            pidConfig.kG = ffkG.get();
            pidConfig.kS = ffkS.get();

            leftMotor.getConfigurator().apply(pidConfig);
            rightMotor.getConfigurator().apply(pidConfig);
            leftMotor.getConfigurator().apply(profileConfig);
            rightMotor.getConfigurator().apply(profileConfig);
        }
    } 

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftMotor.updateFrom(leftMotor);
        inputs.rightMotor.updateFrom(rightMotor);

        updateTunables();

        Logger.recordOutput("Shooter/Left Motor/Output", leftMotor.getClosedLoopOutput().getValueAsDouble());
        Logger.recordOutput("Shooter/Left Motor/FF Out", leftMotor.getClosedLoopFeedForward().getValueAsDouble());
        Logger.recordOutput("Shooter/Left Motor/P Out", leftMotor.getClosedLoopProportionalOutput().getValueAsDouble());
        Logger.recordOutput("Shooter/Left Motor/Profile Velocity", leftMotor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("Shooter/Left Motor/Profile Accleration", leftMotor.getClosedLoopReferenceSlope().getValueAsDouble());
        Logger.recordOutput("Shooter/Right Motor/Output", rightMotor.getClosedLoopOutput().getValueAsDouble());
        Logger.recordOutput("Shooter/Right Motor/FF Out", rightMotor.getClosedLoopFeedForward().getValueAsDouble());
        Logger.recordOutput("Shooter/Right Motor/P Out", rightMotor.getClosedLoopProportionalOutput().getValueAsDouble());
        Logger.recordOutput("Shooter/Right Motor/Profile Velocity", rightMotor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("Shooter/Right Motor/Profile Accleration", rightMotor.getClosedLoopReferenceSlope().getValueAsDouble());
    }

    private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(
        0,
        0,
        false,
        0,
        0,
        false,
        false,
        false
    );

    @Override
    public void setLeftSurfaceSpeed(double rps) {
        leftMotor.setControl(request.withVelocity(rps));
    }

    @Override
    public void setRightSurfaceSpeed(double rps) {
        rightMotor.setControl(request.withVelocity(rps));
    }

    @Override
    public void stop() {
        leftMotor.disable();
        rightMotor.disable();
    }
}
