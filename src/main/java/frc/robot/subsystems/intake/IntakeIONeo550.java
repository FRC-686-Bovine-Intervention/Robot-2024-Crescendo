// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIONeo550 implements IntakeIO {
    private final CANSparkMax rollerMotor = null;
    private final CANSparkMax beltMotor = null;

    private final DigitalInput bottomSensor = new DigitalInput(0);
    private final DigitalInput topSensor = new DigitalInput(1);
    
    public IntakeIONeo550() {
        
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.beltMotor.updateFrom(beltMotor);
        inputs.rollerMotor.updateFrom(rollerMotor);

        inputs.noteAtBottom = bottomSensor.get();
        inputs.noteAtTop = topSensor.get();
    }
    
    @Override
    public void setRollerVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }

    @Override
    public void setBeltVoltage(double voltage) {
        beltMotor.setVoltage(voltage);
    }
}
