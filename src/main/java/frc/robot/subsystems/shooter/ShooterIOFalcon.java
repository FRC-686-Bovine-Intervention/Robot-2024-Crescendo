// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterIOFalcon implements ShooterIO {
    private final TalonFX shooterMotor;
    private final DigitalInput sensor;

    public ShooterIOFalcon() {
        shooterMotor = null;
        sensor = new DigitalInput(0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.notePresent = sensor.get();
        inputs.shooterAppliedVolts = shooterMotor.getSupplyVoltage().getValue();
        inputs.shooterCurrentAmps = shooterMotor.getSupplyCurrent().getValue();
        inputs.shooterTempCelcius = shooterMotor.getDeviceTemp().getValue();
    }
}
