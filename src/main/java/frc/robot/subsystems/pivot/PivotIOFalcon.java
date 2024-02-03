// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class PivotIOFalcon implements PivotIO {
    private final TalonFX pivotMotor = null;
    private final CANcoder pivotEncoder = null;
    
    public PivotIOFalcon() {

    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotMotor.updateFrom(pivotMotor);
        inputs.pivotEncoder.updateFrom(pivotEncoder);
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }
}
