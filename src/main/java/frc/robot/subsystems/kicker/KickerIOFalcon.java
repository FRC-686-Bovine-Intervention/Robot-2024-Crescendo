// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class KickerIOFalcon implements KickerIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final DigitalInput sensor;

    public KickerIOFalcon() {
        leftMotor = null;
        rightMotor = null;

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        sensor = new DigitalInput(0);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.notePresent = sensor.get();

        inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftMotor.getVelocity().getValue());
        inputs.leftAppliedVolts = leftMotor.getSupplyVoltage().getValue();
        inputs.leftCurrentAmps = leftMotor.getSupplyCurrent().getValue();
        inputs.leftTempCelcius = leftMotor.getDeviceTemp().getValue();

        inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightMotor.getVelocity().getValue());
        inputs.rightAppliedVolts = rightMotor.getSupplyVoltage().getValue();
        inputs.rightCurrentAmps = rightMotor.getSupplyCurrent().getValue();
        inputs.rightTempCelcius = rightMotor.getDeviceTemp().getValue();
    }

    @Override
    public void setKickerVoltage(double volts) {
        leftMotor.setVoltage(volts);
    }
}
