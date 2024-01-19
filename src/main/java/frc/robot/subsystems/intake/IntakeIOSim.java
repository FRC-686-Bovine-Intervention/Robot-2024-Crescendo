// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim intakeMotor = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
    private final DCMotorSim beltMotor = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);;

    private final BooleanSupplier noteAtBottom;
    private final BooleanSupplier noteAtTop;

    private double intakeAppliedVolts = 0;
    private double beltAppliedVolts = 0;

    public IntakeIOSim(BooleanSupplier noteAtBottom, BooleanSupplier noteAtTop) {
        this.noteAtBottom = noteAtBottom;
        this.noteAtTop = noteAtTop;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        intakeMotor.update(Constants.dtSeconds);
        beltMotor.update(Constants.dtSeconds);
        
        inputs.intakeVelocityRadPerSec = intakeMotor.getAngularVelocityRadPerSec();
        inputs.intakeCurrentAmps = intakeMotor.getCurrentDrawAmps();
        inputs.intakeAppliedVolts = intakeAppliedVolts;
        inputs.intakeTempCelcius = 0;

        inputs.beltVelocityRadPerSec = beltMotor.getAngularVelocityRadPerSec();
        inputs.beltCurrentAmps = beltMotor.getCurrentDrawAmps();
        inputs.beltAppliedVolts = beltAppliedVolts;
        inputs.beltTempCelcius = 0;
        
        inputs.noteAtBottom = noteAtBottom.getAsBoolean();
        inputs.noteAtTop = noteAtTop.getAsBoolean();
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeAppliedVolts = MathUtil.clamp(volts, -12, 12);

        intakeMotor.setInputVoltage(intakeAppliedVolts);
    }

    @Override
    public void setBeltVoltage(double volts) {
        beltAppliedVolts = MathUtil.clamp(volts, -12, 12);

        beltMotor.setInputVoltage(beltAppliedVolts);
    }
}
