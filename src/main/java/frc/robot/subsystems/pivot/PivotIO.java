// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double pivotPositionRad;
        public double pivotVelocityRadPerSec;
        public double pivotAppliedVolts;
        public double pivotCurrentAmps;
        public double pivotTempCelcius;
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setPivotVoltage(double volts) {}    
}
