// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.LoggedTunableNumber;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim leftMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1);
    private final FlywheelSim rightMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1);
    
    private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/PID/kP", 0);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/PID/kI", 0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/PID/kD", 0);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/PID/kV", 0);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/PID/kA", 0);

    private final ProfiledPIDController leftMotorPID = new ProfiledPIDController(
        kP.get(),
        kI.get(),
        kD.get(),
        new Constraints(
            kV.get(),
            kA.get()
        )
    );

    private final ProfiledPIDController rightMotorPID = new ProfiledPIDController(
        kP.get(),
        kI.get(),
        kD.get(),
        new Constraints(
            kV.get(),
            kA.get()
        )
    );

    private double leftAppliedVolts = 0;
    private double rightAppliedVolts = 0;

    private void updateTunables() {
        if(kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            leftMotorPID.setPID(kP.get(), kI.get(), kD.get());
        }
        if(kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
            leftMotorPID.setConstraints(new Constraints(kV.get(), kA.get()));
        }
    } 

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftMotor.updateFrom(leftMotor, leftAppliedVolts);
        inputs.rightMotor.updateFrom(rightMotor, rightAppliedVolts);

        updateTunables();
    }

    @Override
    public void setLeftVoltage(double rps) {
        var volts = leftMotorPID.calculate(Units.radiansToRotations(leftMotor.getAngularVelocityRadPerSec()), rps);
        leftAppliedVolts = MathUtil.clamp(volts, -12, 12);
            
        leftMotor.setInputVoltage(leftAppliedVolts);
    }

    @Override
    public void setRightVoltage(double rps) {
        var volts = rightMotorPID.calculate(Units.radiansToRotations(rightMotor.getAngularVelocityRadPerSec()), rps);
        rightAppliedVolts = MathUtil.clamp(volts, -12, 12);
        
        rightMotor.setInputVoltage(rightAppliedVolts);
    }
}
