// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    @AutoLog
    public static class KickerIOInputs {
        public boolean notePresent;

        public double kickedAppliedVolts;
        public double kickerCurrentAmps;
        public double kickerTempCelcius;
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setKickerVoltage(double volts) {}
}
