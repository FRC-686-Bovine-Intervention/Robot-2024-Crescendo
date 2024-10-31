// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ShooterIOSim extends ShooterIOFalcon {
    private final DCMotorSim leftMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.005);
    private final DCMotorSim rightMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.005);

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var leftSimState = leftMotor.getSimState();
        var rightSimState = rightMotor.getSimState();
        leftMotorSim.setInputVoltage(leftSimState.getMotorVoltage());
        rightMotorSim.setInputVoltage(rightSimState.getMotorVoltage());

        leftMotorSim.update(Constants.dtSeconds);
        rightMotorSim.update(Constants.dtSeconds);

        var leftPosition = leftMotorSim.getAngularPositionRotations();
        var rightPosition = rightMotorSim.getAngularPositionRotations();
        var leftVelocity = Units.radiansToRotations(leftMotorSim.getAngularVelocityRadPerSec());
        var rightVelocity = Units.radiansToRotations(rightMotorSim.getAngularVelocityRadPerSec());

        leftSimState.setRawRotorPosition(leftPosition);
        rightSimState.setRawRotorPosition(rightPosition);
        leftSimState.setRotorVelocity(leftVelocity);
        rightSimState.setRotorVelocity(rightVelocity);

        leftSimState.setSupplyVoltage(12 - leftSimState.getSupplyCurrent() * 0.002);
        rightSimState.setSupplyVoltage(12 - rightSimState.getSupplyCurrent() * 0.002);
        
        super.updateInputs(inputs);
    }
}
