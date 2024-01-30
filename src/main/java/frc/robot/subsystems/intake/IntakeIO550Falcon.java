// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIO550Falcon implements IntakeIO {
    private final CANSparkMax intakeMotor;
    private final TalonFX beltMotor;

    private final DigitalInput bottomSensor;
    private final DigitalInput topSensor;
    
    // set up the motors with the CAN ID
    public IntakeIO550Falcon() {
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

        inputs.beltVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(beltMotor.getVelocity().getValue());
        inputs.beltAppliedVolts = beltMotor.getSupplyVoltage().getValue();
        inputs.beltCurrentAmps = beltMotor.getSupplyCurrent().getValue();
        inputs.beltTempCelcius = beltMotor.getDeviceTemp().getValue();

        inputs.noteAtBottom = !bottomSensor.get();
        inputs.noteAtTop = !topSensor.get();
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
