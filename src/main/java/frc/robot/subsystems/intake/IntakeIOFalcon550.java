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
    private final CANSparkMax popUpMotor = new CANSparkMax(CANDevices.intakeRollerMotorID, MotorType.kBrushless);
    private final TalonFX intakeMotor = new TalonFX(CANDevices.intakeBeltMotorID, CANDevices.driveCanBusName);


    private final DigitalInput intakeSensor = new DigitalInput(DIOPorts.intakeSensorPort);
    private final DigitalInput kickerSensor = new DigitalInput(DIOPorts.kickerSensorPort);
    
    public IntakeIOFalcon550() {
        popUpMotor.setInverted(true);
        popUpMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        var beltConfig = new TalonFXConfiguration();
        beltConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        beltConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.125;
        intakeMotor.getConfigurator().apply(beltConfig);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.popUpMotor.updateFrom(popUpMotor);
        inputs.intakeMotor.updateFrom(intakeMotor);

        inputs.intakeSensor = !intakeSensor.get();
        inputs.kickerSensor = !kickerSensor.get();
    }
    
    @Override
    public void setPopUpVoltage(double voltage) {
        popUpMotor.setVoltage(voltage);
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }
}
