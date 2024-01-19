// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    @AutoLog
    public static class KickerIOInputs {
        public boolean notePresent;

        public double leftVelocityRadPerSec;
        public double leftAppliedVolts;
        public double leftCurrentAmps;
        public double leftTempCelcius;

        public double rightVelocityRadPerSec;
        public double rightAppliedVolts;
        public double rightCurrentAmps;
        public double rightTempCelcius;
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setKickerVoltage(double volts) {}
}
