// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIONeo550 implements IntakeIO {
    private final CANSparkMax intakeMotor;
    private final CANSparkMax beltMotor;

    private final DigitalInput bottomSensor;
    private final DigitalInput topSensor;
    
    // set up the motors with the CAN ID
    public IntakeIONeo550() {
        intakeMotor = null;
        beltMotor = null;

        bottomSensor = new DigitalInput(0);
        topSensor = new DigitalInput(1);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(intakeMotor.getEncoder().getVelocity());
        inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput();
        inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.intakeTempCelcius = intakeMotor.getMotorTemperature();

        inputs.beltVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(beltMotor.getEncoder().getVelocity());
        inputs.beltAppliedVolts = beltMotor.getAppliedOutput();
        inputs.beltCurrentAmps = beltMotor.getOutputCurrent();
        inputs.beltTempCelcius = beltMotor.getMotorTemperature();

        inputs.noteAtBottom = bottomSensor.get();
        inputs.noteAtTop = topSensor.get();
    }
    
    @Override
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    @Override
    public void setBeltVoltage(double voltage) {
        beltMotor.setVoltage(voltage);
    }
}
