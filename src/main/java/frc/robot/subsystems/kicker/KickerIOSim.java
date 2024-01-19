// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class KickerIOSim implements KickerIO {
    private final DCMotorSim leftMotor = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
    private final DCMotorSim rightMotor = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);

    private final BooleanSupplier notePresent;

    private double leftAppliedVolts = 0;
    private double rightAppliedVolts = 0;
    
    public KickerIOSim(BooleanSupplier notePresent) {
        this.notePresent = notePresent;
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        leftMotor.update(Constants.dtSeconds);
        rightMotor.update(Constants.dtSeconds);
        
        inputs.notePresent = notePresent.getAsBoolean();

        inputs.leftVelocityRadPerSec = leftMotor.getAngularVelocityRadPerSec();
        inputs.leftCurrentAmps = leftMotor.getCurrentDrawAmps();
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftTempCelcius = 0; 

        inputs.rightVelocityRadPerSec = rightMotor.getAngularVelocityRadPerSec();
        inputs.rightCurrentAmps = rightMotor.getCurrentDrawAmps();
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightTempCelcius = 0;
    }

    @Override
    public void setKickerVoltage(double volts) {
        leftAppliedVolts = MathUtil.clamp(volts, -12, 12);
        rightAppliedVolts = MathUtil.clamp(volts, -12, 12);
        
        leftMotor.setInputVoltage(leftAppliedVolts);
        rightMotor.setInputVoltage(rightAppliedVolts);
    }
}
