// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.LoggedTunableNumber;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim leftMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1);
    private final FlywheelSim rightMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1);
    
    private final BooleanSupplier notePresent;

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

    private void updateTunables() {
        if(kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            leftMotorPID.setPID(kP.get(), kI.get(), kD.get());
        }
        if(kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
            leftMotorPID.setConstraints(new Constraints(kV.get(), kA.get()));
        }
    } 

    public ShooterIOSim(BooleanSupplier notePresent) {
        this.notePresent = notePresent;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.notePresent = notePresent.getAsBoolean();

        inputs.leftRotationsPerSecond = Units.radiansToRotations(leftMotor.getAngularVelocityRadPerSec());
        inputs.leftCurrentAmps = leftMotor.getCurrentDrawAmps();
        inputs.leftAppliedVolts = 0;
        inputs.leftTempCelcius = 0;

        inputs.rightRotationsPerSecond = Units.radiansToRotations(rightMotor.getAngularVelocityRadPerSec());
        inputs.rightCurrentAmps = rightMotor.getCurrentDrawAmps();
        inputs.rightAppliedVolts = 0;
        inputs.rightTempCelcius = 0;

        updateTunables();
    }

    @Override
    public void setLeftVelocity(double rps) {
        leftMotor.setInputVoltage(leftMotorPID.calculate(Units.radiansToRotations(leftMotor.getAngularVelocityRadPerSec()), rps));
    }

    @Override
    public void setRightVelocity(double rps) {
        rightMotor.setInputVoltage(rightMotorPID.calculate(Units.radiansToRotations(rightMotor.getAngularVelocityRadPerSec()), rps));
    }
}
