// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim leftMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1);
    private final FlywheelSim rightMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1);
    
    private final BooleanSupplier notePresent;

    public ShooterIOSim(BooleanSupplier notePresent) {
        this.notePresent = notePresent;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.notePresent = notePresent.getAsBoolean();

        inputs.leftRotationsPerSecond = Units.radiansToRotations(leftMotor.getAngularVelocityRadPerSec());
        inputs.leftCurrentAmps = leftMotor.getCurrentDrawAmps();
        inputs.leftAppliedVolts = 0;
        inputs.leftTempCelcius = 0;

        inputs.rightRotationsPerSecond = Units.radiansToRotations(rightMotor.getAngularVelocityRadPerSec());
        inputs.rightCurrentAmps = rightMotor.getCurrentDrawAmps();
        inputs.rightAppliedVolts = 0;
        inputs.rightTempCelcius = 0;
    }

    @Override
    public void setLeftVoltage(double volts) {
        leftMotor.setInputVoltage(volts);
    }

    @Override
    public void setRightVoltage(double volts) {
        rightMotor.setInputVoltage(volts);
    }
}
