// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getFalcon500(1).withReduction(25), 4, 1, Units.inchesToMeters(11.876), 0, Units.degreesToRadians(41.353), false, 0);

    public PivotIOSim() {}

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        pivotSim.update(0.02);

        inputs.pivotPositionRad = pivotSim.getAngleRads();
        inputs.pivotVelocityRadPerSec = pivotSim.getVelocityRadPerSec();
        inputs.pivotCurrentAmps = pivotSim.getCurrentDrawAmps();
        inputs.pivotAppliedVolts = 0;
        inputs.pivotTempCelcius = 0;
    }
    
    @Override
    public void setPivotVoltage(double volts) {
        pivotSim.setInputVoltage(volts);
    }
}
