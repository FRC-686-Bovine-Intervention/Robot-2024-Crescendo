// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.CANDevices;
import frc.robot.util.LoggedTunableNumber;

public class ShooterIOFalcon implements ShooterIO {
    private final TalonFX leftMotor = new TalonFX(CANDevices.shooterLeftID);
    private final TalonFX rightMotor = new TalonFX(CANDevices.shooterRightID);

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/PID/kP", 0);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/PID/kI", 0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/PID/kD", 0);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/PID/Profile/kA", 0);
    private final LoggedTunableNumber kJ = new LoggedTunableNumber("Shooter/PID/Profile/kJ", 0);
    private final LoggedTunableNumber ffkV = new LoggedTunableNumber("Shooter/FF/kV", 0);
    private final LoggedTunableNumber ffkA = new LoggedTunableNumber("Shooter/FF/kA", 0);
    private final LoggedTunableNumber ffkG = new LoggedTunableNumber("Shooter/FF/kG", 0);
    private final LoggedTunableNumber ffkS = new LoggedTunableNumber("Shooter/FF/kS", 0);

    public ShooterIOFalcon() {
        var config = new TalonFXConfiguration();
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotor.getConfigurator().apply(config);
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightMotor.getConfigurator().apply(config);
        updateTunables();
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
            leftMotor.getConfigurator().apply(pidConfig);
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
    public void setLeftVelocity(double rps) {
        request.withVelocity(rps);
        leftMotor.setControl(request);
    }

    @Override
    public void setRightVelocity(double rps) {
        request.withVelocity(rps);
        rightMotor.setControl(request);
    }

    @Override
    public void stop() {
        leftMotor.setControl(new NeutralOut());
        rightMotor.setControl(new NeutralOut());
    }
}
