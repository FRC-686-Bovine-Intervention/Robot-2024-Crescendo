// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;

public class ShooterIOSim implements ShooterIO {
    private final TalonFX leftTalonFX = new TalonFX(CANDevices.shooterLeftID);
    private final TalonFX rightTalonFX = new TalonFX(CANDevices.shooterRightID);
    private final DCMotorSim leftMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.005);
    private final DCMotorSim rightMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.005);

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/PID/kP", 0.005);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/PID/kI", 0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/PID/kD", 0);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/PID/Profile/kA", 120);
    private final LoggedTunableNumber kJ = new LoggedTunableNumber("Shooter/PID/Profile/kJ", 120);
    private final LoggedTunableNumber ffkV = new LoggedTunableNumber("Shooter/FF/kV", 0.4);
    private final LoggedTunableNumber ffkA = new LoggedTunableNumber("Shooter/FF/kA", 0);
    private final LoggedTunableNumber ffkG = new LoggedTunableNumber("Shooter/FF/kG", 0);
    private final LoggedTunableNumber ffkS = new LoggedTunableNumber("Shooter/FF/kS", 0);

    public ShooterIOSim() {
        var config = new TalonFXConfiguration();
        // config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.motorToSurface.surfacePerRot();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftTalonFX.getConfigurator().apply(config);
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightTalonFX.getConfigurator().apply(config);
        updateTunables();

        leftTalonFX.getRotorVelocity().setUpdateFrequency(50);
        leftTalonFX.getClosedLoopOutput().setUpdateFrequency(50);
        leftTalonFX.getClosedLoopReference().setUpdateFrequency(50);
        rightTalonFX.getRotorVelocity().setUpdateFrequency(50);
        leftTalonFX.getStatorCurrent().setUpdateFrequency(50);
        rightTalonFX.getStatorCurrent().setUpdateFrequency(50);
        leftTalonFX.getClosedLoopReferenceSlope().setUpdateFrequency(50);
    }

    private void updateTunables() {
        if(
            kP.hasChanged(hashCode()) |
            kI.hasChanged(hashCode()) |
            kD.hasChanged(hashCode()) |
            kA.hasChanged(hashCode()) |
            kJ.hasChanged(hashCode()) |
            ffkV.hasChanged(hashCode()) |
            ffkA.hasChanged(hashCode()) |
            ffkG.hasChanged(hashCode()) |
            ffkS.hasChanged(hashCode())
        ) {
            var pidConfig = new Slot0Configs();
            var profileConfig = new MotionMagicConfigs();
            pidConfig.kP = kP.get();
            pidConfig.kI = kI.get();
            pidConfig.kD = kD.get();
            profileConfig.MotionMagicAcceleration = kA.get();
            profileConfig.MotionMagicJerk = kJ.get();
            pidConfig.kV = ffkV.get();
            pidConfig.kA = ffkA.get();
            pidConfig.kG = ffkG.get();
            pidConfig.kS = ffkS.get();

            leftTalonFX.getConfigurator().apply(pidConfig);
            rightTalonFX.getConfigurator().apply(pidConfig);
            leftTalonFX.getConfigurator().apply(profileConfig);
            rightTalonFX.getConfigurator().apply(profileConfig);
        }
    } 

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var leftSimState = leftTalonFX.getSimState();
        var rightSimState = rightTalonFX.getSimState();
        leftMotorSim.setInputVoltage(leftSimState.getMotorVoltage());
        rightMotorSim.setInputVoltage(rightSimState.getMotorVoltage());

        leftMotorSim.update(Constants.dtSeconds);
        rightMotorSim.update(Constants.dtSeconds);

        var leftPosition = leftMotorSim.getAngularPositionRotations();
        var rightPosition = rightMotorSim.getAngularPositionRotations();
        var leftVelocity = Units.radiansToRotations(leftMotorSim.getAngularVelocityRadPerSec());
        var rightVelocity = Units.radiansToRotations(rightMotorSim.getAngularVelocityRadPerSec());

        leftSimState.setRawRotorPosition(leftPosition);
        rightSimState.setRawRotorPosition(rightPosition);
        leftSimState.setRotorVelocity(leftVelocity);
        rightSimState.setRotorVelocity(rightVelocity);

        leftSimState.setSupplyVoltage(12 - leftSimState.getSupplyCurrent() * 0.002);
        rightSimState.setSupplyVoltage(12 - rightSimState.getSupplyCurrent() * 0.002);

        inputs.leftMotor.updateFrom(leftTalonFX);
        inputs.rightMotor.updateFrom(rightTalonFX);

        updateTunables();

        Logger.recordOutput("Shooter/Left Motor/Output", leftTalonFX.getClosedLoopOutput().getValueAsDouble());
        Logger.recordOutput("Shooter/Left Motor/FF Out", leftTalonFX.getClosedLoopFeedForward().getValueAsDouble());
        Logger.recordOutput("Shooter/Left Motor/P Out", leftTalonFX.getClosedLoopProportionalOutput().getValueAsDouble());
        Logger.recordOutput("Shooter/Left Motor/Profile Velocity", leftTalonFX.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("Shooter/Left Motor/Profile Accleration", leftTalonFX.getClosedLoopReferenceSlope().getValueAsDouble());
        Logger.recordOutput("Shooter/Right Motor/Output", rightTalonFX.getClosedLoopOutput().getValueAsDouble());
        Logger.recordOutput("Shooter/Right Motor/FF Out", rightTalonFX.getClosedLoopFeedForward().getValueAsDouble());
        Logger.recordOutput("Shooter/Right Motor/P Out", rightTalonFX.getClosedLoopProportionalOutput().getValueAsDouble());
        Logger.recordOutput("Shooter/Right Motor/Profile Velocity", rightTalonFX.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("Shooter/Right Motor/Profile Accleration", rightTalonFX.getClosedLoopReferenceSlope().getValueAsDouble());
    }

    private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(
        0,
        0,
        false,
        0,
        0,
        false,
        false,
        false
    );

    @Override
    public void setLeftSurfaceSpeed(double rps) {
        leftTalonFX.setControl(request.withVelocity(rps));
    }

    @Override
    public void setRightSurfaceSpeed(double rps) {
        rightTalonFX.setControl(request.withVelocity(rps));
    }

    @Override
    public void stop() {
        leftTalonFX.disable();
        rightTalonFX.disable();
    }
}
