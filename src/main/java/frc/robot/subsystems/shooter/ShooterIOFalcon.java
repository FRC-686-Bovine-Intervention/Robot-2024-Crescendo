// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.util.LoggedTunableNumber;

public class ShooterIOFalcon implements ShooterIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final TalonFXConfiguration motorConfiguration;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/PID/kP", 0);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/PID/kI", 0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/PID/kD", 0);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/PID/kV", 0);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/PID/kA", 0);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Shooter/PID/kG", 0);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/PID/kS", 0);

    public ShooterIOFalcon() {
        leftMotor = null;
        rightMotor = null;

        motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        
        applyMotorConfig();
    }

    private void applyMotorConfig() {
        leftMotor.getConfigurator().apply(motorConfiguration);
        rightMotor.getConfigurator().apply(motorConfiguration);
    }

    private void updateTunables() {
        if(
            kP.hasChanged(hashCode()) ||
            kI.hasChanged(hashCode()) ||
            kD.hasChanged(hashCode()) ||
            kV.hasChanged(hashCode()) ||
            kA.hasChanged(hashCode()) ||
            kG.hasChanged(hashCode()) ||
            kS.hasChanged(hashCode())
        ) {
            motorConfiguration.Slot0.kP = kP.get();
            motorConfiguration.Slot0.kI = kI.get();
            motorConfiguration.Slot0.kD = kD.get();
            motorConfiguration.Slot0.kV = kV.get();
            motorConfiguration.Slot0.kA = kA.get();
            motorConfiguration.Slot0.kG = kG.get();
            motorConfiguration.Slot0.kS = kS.get();

            applyMotorConfig();
        }
    } 

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftMotor.updateFrom(leftMotor);
        inputs.rightMotor.updateFrom(rightMotor);

        updateTunables();
    }

    private final boolean kEnableFOC = true;
    private final double kAcceleration = 1;
    private final double kFeedForward = 0;
    private final int kPIDSlot = 0;
    private final boolean kOverrideBrakeDurNeutral = true;
    private final boolean kLimitForwardMotion = true;
    private final boolean kLimitReverseMotion = true;

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
