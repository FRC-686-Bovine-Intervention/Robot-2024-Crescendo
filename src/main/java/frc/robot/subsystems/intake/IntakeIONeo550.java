// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.DIOPorts;

public class IntakeIONeo550 implements IntakeIO {
    private final CANSparkMax rollerMotor = null;
    private final CANSparkMax beltMotor = null;

    private final DigitalInput intakeSensor = new DigitalInput(DIOPorts.intakeSensorPort);
    private final DigitalInput kickerSensor = new DigitalInput(DIOPorts.kickerSensorPort);
    
    public IntakeIONeo550() {
        
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.popUpMotor.updateFrom(beltMotor);
        inputs.intakeMotor.updateFrom(rollerMotor);

        inputs.intakeSensor = !intakeSensor.get();
        inputs.kickerSensor = !kickerSensor.get();
    }
    
    @Override
    public void setPopUpVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        beltMotor.setVoltage(voltage);
    }
}
