// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
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
    private final TalonFX pivotLeftMotor = new TalonFX(CANDevices.pivotLeftMotorID);
    private final TalonFX pivotRightMotor = new TalonFX(CANDevices.pivotRightMotorID);
    private final CANcoder pivotEncoder = new CANcoder(CANDevices.pivotEncoderID);
    
    public PivotIOFalcon() {
        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.RotorToSensorRatio = PivotConstants.motorToEncoderRatio.ratio();
        motorConfig.Feedback.SensorToMechanismRatio = PivotConstants.encoderToMechanismRatio.ratio();
        motorConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Radians.of(Pivot.POS_ZERO).in(Rotations);
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Radians.of(Pivot.POS_AMP).in(Rotations);
        pivotLeftMotor.getConfigurator().apply(motorConfig);
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotRightMotor.getConfigurator().apply(motorConfig);
        
        var encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = PivotConstants.pivotMagnetOffset;
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        pivotEncoder.getConfigurator().apply(encoderConfig);

        pivotRightMotor.setControl(new StrictFollower(CANDevices.pivotLeftMotorID));
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotLeftMotor.updateFrom(pivotLeftMotor);
        inputs.pivotRightMotor.updateFrom(pivotRightMotor);
        inputs.pivotEncoder.updateFrom(pivotEncoder);
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotLeftMotor.setVoltage(volts);
    }
}
