// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOPorts;

public class IntakeIOFalcon550 implements IntakeIO {
    private final CANSparkMax rollerMotor = new CANSparkMax(CANDevices.intakeRollerMotorID, MotorType.kBrushless);
    private final TalonFX beltMotor = new TalonFX(CANDevices.intakeBeltMotorID, CANDevices.driveCanBusName);

    private final DigitalInput sensor = new DigitalInput(DIOPorts.intakeSensorPort);
    
    public IntakeIOFalcon550() {
        rollerMotor.setInverted(false);
        rollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        var beltConfig = new TalonFXConfiguration();
        beltConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        beltConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.125;
        beltMotor.getConfigurator().apply(beltConfig);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.beltMotor.updateFrom(beltMotor);
        inputs.rollerMotor.updateFrom(rollerMotor);

        inputs.sensor = !sensor.get();
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
