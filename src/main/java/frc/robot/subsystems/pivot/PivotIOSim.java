// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class PivotIOSim extends PivotIOFalcon {
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(2).withReduction(25),
        4,
        0.5,
        Units.inchesToMeters(11.876),
        -Units.degreesToRadians(Pivot.idleAltitudeDeg.get()),
        Units.degreesToRadians(Pivot.ampAltitudeDeg.get()),
        true,
        0
    );

    public PivotIOSim() {
        super();
        var encoderConfig = new CANcoderConfiguration();
        // pivotEncoder.getConfigurator().refresh(encoderConfig);
        encoderConfig.MagnetSensor.MagnetOffset = 0;
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        pivotEncoder.getConfigurator().apply(encoderConfig);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        var leftSimState = pivotLeftMotor.getSimState();
        var rightSimState = pivotRightMotor.getSimState();
        var encoderSimState = pivotEncoder.getSimState();
        pivotSim.setInputVoltage(-leftSimState.getMotorVoltage());

        pivotSim.update(Constants.dtSeconds);

        Logger.recordOutput("Pivot/SIM_angle", pivotSim.getAngleRads());
        var position = Units.radiansToRotations(pivotSim.getAngleRads())+0.25;
        var velocity = Units.radiansToRotations(pivotSim.getVelocityRadPerSec());

        encoderSimState.setRawPosition(position);
        encoderSimState.setVelocity(velocity);

        leftSimState.setSupplyVoltage(12 - leftSimState.getSupplyCurrent() * 0.002);
        rightSimState.setSupplyVoltage(12 - rightSimState.getSupplyCurrent() * 0.002);

        super.updateInputs(inputs);
    }
}
