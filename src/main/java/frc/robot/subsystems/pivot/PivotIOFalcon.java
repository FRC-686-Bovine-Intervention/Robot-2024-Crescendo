// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;

public class PivotIOFalcon implements PivotIO {
    private final TalonFX pivotMotor = null;
    private final CANcoder pivotEncoder = null;
    
    // set up the motor and encoder with the CAN ID
    public PivotIOFalcon() {
        // 
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotPositionRad = Units.rotationsToRadians(pivotEncoder.getPosition().getValue());
        inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotEncoder.getVelocity().getValue());
        inputs.pivotAppliedVolts = pivotMotor.getSupplyVoltage().getValue();
        inputs.pivotCurrentAmps = pivotMotor.getSupplyCurrent().getValue();
        inputs.pivotTempCelcius = pivotMotor.getDeviceTemp().getValue();
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }
}
