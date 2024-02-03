// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.DIOPorts;

public class KickerIOFalcon implements KickerIO {
    private final TalonFX leftMotor = null;
    private final TalonFX rightMotor = null;

    private final DigitalInput sensor = new DigitalInput(DIOPorts.kickerSensorPort);

    public KickerIOFalcon() {
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.notePresent = sensor.get();

        inputs.leftMotor.updateFrom(leftMotor);
        inputs.rightMotor.updateFrom(rightMotor);
    }

    @Override
    public void setKickerVoltage(double volts) {
        leftMotor.setVoltage(volts);
    }
}
