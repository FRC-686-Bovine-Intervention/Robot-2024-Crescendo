// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim shooterMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1);
    private final BooleanSupplier notePresent;

    public ShooterIOSim(BooleanSupplier notePresent) {
        this.notePresent = notePresent;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.notePresent = notePresent.getAsBoolean();
        inputs.shooterCurrentAmps = shooterMotor.getCurrentDrawAmps();
        inputs.shooterAppliedVolts = 0;
        inputs.shooterTempCelcius = 0;
    }

    @Override
    public void setShooterVoltage(double volts) {
        shooterMotor.setInputVoltage(volts);
    }
}
