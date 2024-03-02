// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.loggerUtil.LoggedMotor;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean intakeSensor;
        public boolean kickerSensor;

        public LoggedMotor popUpMotor = new LoggedMotor();
        public LoggedMotor intakeMotor = new LoggedMotor();
        public LoggedMotor kickerMotor = new LoggedMotor();
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setPopUpVoltage(double voltage) {}

    public default void setIntakeVoltage(double voltage) {}

    public default void setKickerVoltage(double voltage) {}
}
