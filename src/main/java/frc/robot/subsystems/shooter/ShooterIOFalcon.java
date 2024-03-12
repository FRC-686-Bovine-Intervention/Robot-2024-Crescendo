// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.CANDevices;

public class ShooterIOFalcon implements ShooterIO {
    private final TalonFX leftMotor = new TalonFX(CANDevices.shooterLeftID);
    private final TalonFX rightMotor = new TalonFX(CANDevices.shooterRightID);

    public ShooterIOFalcon() {
        var config = new TalonFXConfiguration();
        // config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotor.getConfigurator().apply(config);
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftMotor.updateFrom(leftMotor);
        inputs.rightMotor.updateFrom(rightMotor);
    }
    
    @Override
    public void setLeftVoltage(double volts) {
        leftMotor.setVoltage(MathUtil.clamp(volts, -Shooter.maxVolts.get(), Shooter.maxVolts.get()));
    }

    @Override
    public void setRightVoltage(double volts) {
        rightMotor.setVoltage(MathUtil.clamp(volts, -Shooter.maxVolts.get(), Shooter.maxVolts.get()));;
    }
}
