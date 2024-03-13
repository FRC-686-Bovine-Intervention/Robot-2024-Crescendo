// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/PID/kP", 0.7*12);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/PID/kI", 0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/PID/kD", 0);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/PID/Profile/kA", 0);
    private final LoggedTunableNumber kJ = new LoggedTunableNumber("Shooter/PID/Profile/kJ", 0);
    private final LoggedTunableNumber ffkV = new LoggedTunableNumber("Shooter/FF/kV", 0.02*12);
    private final LoggedTunableNumber ffkA = new LoggedTunableNumber("Shooter/FF/kA", 0);
    private final LoggedTunableNumber ffkG = new LoggedTunableNumber("Shooter/FF/kG", 0);
    private final LoggedTunableNumber ffkS = new LoggedTunableNumber("Shooter/FF/kS", 0);

    public ShooterIOFalcon() {
        var config = new TalonFXConfiguration();
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
        updateTunables(config);
    }

    private void applyMotorConfig(TalonFXConfiguration config) {
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotor.getConfigurator().apply(config);
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightMotor.getConfigurator().apply(config);
    }

    private void updateTunables(TalonFXConfiguration config) {
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
            if(config == null) config = new TalonFXConfiguration();
            config.Slot0.kP = kP.get();
            config.Slot0.kI = kI.get();
            config.Slot0.kD = kD.get();
            config.MotionMagic.MotionMagicAcceleration = kA.get();
            config.MotionMagic.MotionMagicJerk = kJ.get();
            config.Slot0.kV = ffkV.get();
            config.Slot0.kA = ffkA.get();
            config.Slot0.kG = ffkG.get();
            config.Slot0.kS = ffkS.get();

            applyMotorConfig(config);
        }
    } 

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftMotor.updateFrom(leftMotor);
        inputs.rightMotor.updateFrom(rightMotor);

        updateTunables(null);
    }

    private final boolean kEnableFOC = false;
    private final double kAcceleration = 1;
    private final double kFeedForward = 0;
    private final int kPIDSlot = 0;
    private final boolean kOverrideBrakeDurNeutral = false;
    private final boolean kLimitForwardMotion = false;
    private final boolean kLimitReverseMotion = false;

    private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(
        0,
        0,
        kEnableFOC,
        kFeedForward,
        kPIDSlot,
        kOverrideBrakeDurNeutral,
        kLimitForwardMotion,
        kLimitReverseMotion
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
