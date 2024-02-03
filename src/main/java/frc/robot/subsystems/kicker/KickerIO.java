// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.loggerUtil.LoggedMotor;

public interface KickerIO {
    @AutoLog
    public static class KickerIOInputs {
        public boolean notePresent;

        public LoggedMotor leftMotor = new LoggedMotor();
        public LoggedMotor rightMotor = new LoggedMotor();
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setKickerVoltage(double volts) {}
}
