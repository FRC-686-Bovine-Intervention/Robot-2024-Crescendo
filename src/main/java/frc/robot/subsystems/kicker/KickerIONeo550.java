// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOPorts;

public class KickerIONeo550 implements KickerIO {
    private final CANSparkMax leftMotor = new CANSparkMax(CANDevices.kickerLeftID, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(CANDevices.kickerRightID, MotorType.kBrushless);

    private final DigitalInput sensor = new DigitalInput(DIOPorts.kickerSensorPort);
    private final Debouncer sensorDebouncer = new Debouncer(Constants.dtSeconds * 3, DebounceType.kBoth);

    public KickerIONeo550() {
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        rightMotor.follow(leftMotor, true);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.notePresent = sensorDebouncer.calculate(!sensor.get());

        inputs.leftMotor.updateFrom(leftMotor);
        inputs.rightMotor.updateFrom(rightMotor);
    }

    @Override
    public void setKickerVoltage(double volts) {
        leftMotor.setVoltage(volts);
    }
}
