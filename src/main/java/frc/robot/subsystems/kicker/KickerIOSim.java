// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class KickerIOSim implements KickerIO {
    private final DCMotorSim kickerMotor = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
    private final BooleanSupplier notePresent;
    
    public KickerIOSim(BooleanSupplier notePresent) {
        this.notePresent = notePresent;
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        kickerMotor.update(Constants.dtSeconds);
        
        inputs.notePresent = notePresent.getAsBoolean();
        inputs.kickerCurrentAmps = kickerMotor.getCurrentDrawAmps();
        inputs.kickedAppliedVolts = 0;
        inputs.kickerTempCelcius = 0; 
    }

    @Override
    public void setKickerVoltage(double volts) {
        kickerMotor.setInputVoltage(volts);
    }
}
