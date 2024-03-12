// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.CANDevices;
import frc.robot.util.LoggedTunableNumber;

public class ShooterIOFalcon implements ShooterIO {
    private final TalonFX leftMotor = new TalonFX(CANDevices.shooterLeftID);
    private final TalonFX rightMotor = new TalonFX(CANDevices.shooterRightID);

    private final TalonFXConfiguration leftConfiguration = new TalonFXConfiguration();
    private final TalonFXConfiguration rightConfiguration = new TalonFXConfiguration();

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/PID/kP", 0.03);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/PID/kI", 0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/PID/kD", 0);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/PID/kV", 0.011);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/PID/kA", 0);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Shooter/PID/kG", 0);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/PID/kS", 0);

    public ShooterIOFalcon() {
        leftConfiguration.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        rightConfiguration.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        leftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
        rightConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
        applyMotorConfig();
    }

    private void applyMotorConfig() {
        leftMotor.getConfigurator().apply(leftConfiguration);
        rightMotor.getConfigurator().apply(rightConfiguration);
    }

    private void updateTunables() {
        if(
            kP.hasChanged(hashCode()) |
            kI.hasChanged(hashCode()) |
            kD.hasChanged(hashCode()) |
            kV.hasChanged(hashCode()) |
            kA.hasChanged(hashCode()) |
            kG.hasChanged(hashCode()) |
            kS.hasChanged(hashCode())
        ) {
            leftConfiguration.Slot0.kP = kP.get();
            leftConfiguration.Slot0.kI = kI.get();
            leftConfiguration.Slot0.kD = kD.get();
            leftConfiguration.Slot0.kV = kV.get();
            leftConfiguration.Slot0.kA = kA.get();
            leftConfiguration.Slot0.kG = kG.get();
            leftConfiguration.Slot0.kS = kS.get();
            rightConfiguration.Slot0.kP = kP.get();
            rightConfiguration.Slot0.kI = kI.get();
            rightConfiguration.Slot0.kD = kD.get();
            rightConfiguration.Slot0.kV = kV.get();
            rightConfiguration.Slot0.kA = kA.get();
            rightConfiguration.Slot0.kG = kG.get();
            rightConfiguration.Slot0.kS = kS.get();

            applyMotorConfig();
        }
    } 

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftMotor.updateFrom(leftMotor);
        inputs.rightMotor.updateFrom(rightMotor);

        updateTunables();
    }

    private final boolean kEnableFOC = false;
    private final double kAcceleration = 1;
    private final double kFeedForward = 0;
    private final int kPIDSlot = 0;
    private final boolean kOverrideBrakeDurNeutral = false;
    private final boolean kLimitForwardMotion = false;
    private final boolean kLimitReverseMotion = false;

    private final VelocityDutyCycle request = new VelocityDutyCycle(
        0,
        kAcceleration,
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
}
