// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PivotConstants;

public class PivotIOFalcon implements PivotIO {
    private final TalonFX pivotMotor = new TalonFX(CANDevices.pivotMotorID);
    private final CANcoder pivotEncoder = new CANcoder(CANDevices.pivotEncoderID);
    
    public PivotIOFalcon() {
        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.RotorToSensorRatio = PivotConstants.motorToEncoderRatio.drivenToDrive();
        motorConfig.Feedback.SensorToMechanismRatio = PivotConstants.encoderToMechanismRatio.drivenToDrive();
        motorConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Radians.of(Pivot.POS_ZERO).in(Rotations);
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Radians.of(Pivot.POS_AMP).in(Rotations);
        pivotMotor.getConfigurator().apply(motorConfig);

        var encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = PivotConstants.pivotMagnetOffset;
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoder.getConfigurator().apply(encoderConfig);
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
