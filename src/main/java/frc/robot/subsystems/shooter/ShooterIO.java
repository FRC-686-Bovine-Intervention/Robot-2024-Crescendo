// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.loggerUtil.LoggedMotor;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean notePresent;

        public LoggedMotor leftMotor = new LoggedMotor();
        public LoggedMotor rightMotor = new LoggedMotor();
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setLeftVelocity(double rps) {}

    public default void setRightVelocity(double rps) {}
}
