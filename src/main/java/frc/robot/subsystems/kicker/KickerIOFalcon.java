// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

public class KickerIOFalcon implements KickerIO {
    private final TalonFX kickerMotor;
    private final DigitalInput sensor;

    public KickerIOFalcon() {
        kickerMotor = null;
        sensor = new DigitalInput(0);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.notePresent = sensor.get();
        inputs.kickedAppliedVolts = kickerMotor.getSupplyVoltage().getValue();
        inputs.kickerCurrentAmps = kickerMotor.getSupplyCurrent().getValue();
        inputs.kickerTempCelcius = kickerMotor.getDeviceTemp().getValue();
    }

    @Override
    public void setKickerVoltage(double volts) {
        kickerMotor.setVoltage(volts);
    }
}
