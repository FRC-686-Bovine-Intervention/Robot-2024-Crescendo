// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim intakeMotor = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
    private final DCMotorSim beltMotor = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);;

    private final BooleanSupplier noteAtBottom;
    private final BooleanSupplier noteAtTop;

    public IntakeIOSim(BooleanSupplier noteAtBottom, BooleanSupplier noteAtTop) {
        this.noteAtBottom = noteAtBottom;
        this.noteAtTop = noteAtTop;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        intakeMotor.update(0.05);
        beltMotor.update(0.05);
        inputs.intakeVelocityRadPerSec = intakeMotor.getAngularVelocityRadPerSec();
        inputs.intakeCurrentAmps = intakeMotor.getCurrentDrawAmps();
        inputs.intakeAppliedVolts = 0;
        inputs.intakeTempCelcius = 0;

        inputs.beltVelocityRadPerSec = beltMotor.getAngularVelocityRadPerSec();
        inputs.beltCurrentAmps = beltMotor.getCurrentDrawAmps();
        inputs.beltAppliedVolts = 0;
        inputs.beltTempCelcius = 0;
        
        inputs.noteAtBottom = noteAtBottom.getAsBoolean();
        inputs.noteAtTop = noteAtTop.getAsBoolean();
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setInputVoltage(voltage);
    }

    @Override
    public void setBeltVoltage(double voltage) {
        beltMotor.setInputVoltage(voltage);
    }
}
