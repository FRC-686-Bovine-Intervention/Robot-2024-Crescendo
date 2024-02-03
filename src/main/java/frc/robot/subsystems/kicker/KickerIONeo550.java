// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.DIOPorts;

public class KickerIONeo550 implements KickerIO {
    private final CANSparkMax leftMotor = null;
    private final CANSparkMax rightMotor = null;

    private final DigitalInput sensor = new DigitalInput(DIOPorts.kickerSensorPort);

    public KickerIONeo550() {
        rightMotor.follow(leftMotor, true);
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
