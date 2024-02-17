// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.CANDevices;

public class PivotIOFalcon implements PivotIO {
    private final TalonFX pivotMotor = null;
    private final CANcoder pivotEncoder = new CANcoder(CANDevices.pivotEncoderID);
    // private final double calibValue = -0.09667968750000001;
    
    public PivotIOFalcon() {
        var a = new CANcoderConfiguration();
        a.MagnetSensor.MagnetOffset = 0.21337890625;//-0.077392578125;
        a.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        a.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoder.getConfigurator().apply(a);
        // pivotEncoder.setPosition(pivotEncoder.getAbsolutePosition().getValueAsDouble() - calibValue);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        // inputs.pivotMotor.updateFrom(pivotMotor);
        inputs.pivotEncoder.updateFrom(pivotEncoder);
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }
}
