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
    private final DCMotorSim rollerMotor = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
    private final DCMotorSim beltMotor = new DCMotorSim(DCMotor.getFalcon500(1), 1, 1);;

    private final BooleanSupplier intakeSensor;
    private final BooleanSupplier kickerSensor;

    private double rollerAppliedVolts = 0;
    private double beltAppliedVolts = 0;

    public IntakeIOSim(BooleanSupplier intakeSensor, BooleanSupplier kickerSensor) {
        this.intakeSensor = intakeSensor;
        this.kickerSensor = kickerSensor;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        rollerMotor.update(Constants.dtSeconds);
        beltMotor.update(Constants.dtSeconds);
        
        inputs.intakeMotor.updateFrom(rollerMotor, rollerAppliedVolts);
        inputs.popUpMotor.updateFrom(beltMotor, beltAppliedVolts);
        
        inputs.intakeSensor = intakeSensor.getAsBoolean();
        inputs.kickerSensor = kickerSensor.getAsBoolean();
    }

    @Override
    public void setPopUpVoltage(double volts) {
        rollerAppliedVolts = MathUtil.clamp(volts, -12, 12);

        rollerMotor.setInputVoltage(rollerAppliedVolts);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        beltAppliedVolts = MathUtil.clamp(volts, -12, 12);

        beltMotor.setInputVoltage(beltAppliedVolts);
    }
}
