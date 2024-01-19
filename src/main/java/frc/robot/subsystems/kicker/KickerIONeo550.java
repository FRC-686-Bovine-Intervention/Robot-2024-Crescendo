// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class KickerIONeo550 implements KickerIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final DigitalInput sensor;

    public KickerIONeo550() {
        leftMotor = null;
        rightMotor = null;

        rightMotor.follow(leftMotor, true);

        sensor = new DigitalInput(0);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.notePresent = sensor.get();

        inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(leftMotor.getEncoder().getVelocity());
        inputs.leftAppliedVolts = leftMotor.getAppliedOutput();
        inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
        inputs.leftTempCelcius = leftMotor.getMotorTemperature();

        inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rightMotor.getEncoder().getVelocity());
        inputs.rightAppliedVolts = rightMotor.getAppliedOutput();
        inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
        inputs.rightTempCelcius = rightMotor.getMotorTemperature();
    }

    @Override
    public void setKickerVoltage(double volts) {
        leftMotor.setVoltage(volts);
    }
}
