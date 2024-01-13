// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean noteAtBottom;
        public boolean noteAtTop;

        public double intakeVelocityRadPerSec;
        public double intakeAppliedVolts;
        public double intakeCurrentAmps;
        public double intakeTempCelcius;

        public double beltVelocityRadPerSec;
        public double beltAppliedVolts;
        public double beltCurrentAmps;
        public double beltTempCelcius;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakeVoltage(double voltage) {}

    public default void setBeltVoltage(double voltage) {}
}
