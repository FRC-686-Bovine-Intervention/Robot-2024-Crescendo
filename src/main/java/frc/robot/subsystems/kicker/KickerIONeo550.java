// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class KickerIONeo550 implements KickerIO {
    private final CANSparkMax kickerMotor;
    private final DigitalInput sensor;

    public KickerIONeo550() {
        kickerMotor = null;
        sensor = new DigitalInput(0);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.notePresent = sensor.get();
        inputs.kickedAppliedVolts = kickerMotor.getAppliedOutput();
        inputs.kickerCurrentAmps = kickerMotor.getOutputCurrent();
        inputs.kickerTempCelcius = kickerMotor.getMotorTemperature();
    }

    @Override
    public void setKickerVoltage(double volts) {
        kickerMotor.setVoltage(volts);
    }
}
