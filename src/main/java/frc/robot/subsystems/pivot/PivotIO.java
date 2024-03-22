// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.loggerUtil.LoggedEncoder;
import frc.robot.util.loggerUtil.LoggedMotor;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public LoggedMotor pivotLeftMotor = new LoggedMotor();
        public LoggedMotor pivotRightMotor = new LoggedMotor();
        public LoggedEncoder pivotEncoder = new LoggedEncoder();
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setPivotVoltage(double volts) {}

    public default void setCoast(boolean coast) {}
}
