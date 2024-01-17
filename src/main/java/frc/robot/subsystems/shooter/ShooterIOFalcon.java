// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterIOFalcon implements ShooterIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final DigitalInput sensor;

    public ShooterIOFalcon() {
        leftMotor = null;
        rightMotor = null;

        sensor = new DigitalInput(0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.notePresent = sensor.get();

        inputs.leftRotationsPerSecond = leftMotor.getVelocity().getValue();
        inputs.leftAppliedVolts = leftMotor.getSupplyVoltage().getValue();
        inputs.leftCurrentAmps = leftMotor.getSupplyCurrent().getValue();
        inputs.leftTempCelcius = leftMotor.getDeviceTemp().getValue();

        inputs.rightRotationsPerSecond = rightMotor.getVelocity().getValue();
        inputs.rightAppliedVolts = rightMotor.getSupplyVoltage().getValue();
        inputs.rightCurrentAmps = rightMotor.getSupplyCurrent().getValue();
        inputs.rightTempCelcius = rightMotor.getDeviceTemp().getValue();
    }

    public void setLeftVoltage(double volts) {
        leftMotor.setVoltage(volts);
    }

    public void setRightVoltage(double volts) {
        rightMotor.setVoltage(volts);
    }
}
