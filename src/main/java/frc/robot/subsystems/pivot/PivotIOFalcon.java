// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOPorts;
import frc.robot.Constants.PivotConstants;
import frc.robot.util.LoggedTunableNumber;

public class PivotIOFalcon implements PivotIO {
    protected final TalonFX pivotLeftMotor = new TalonFX(CANDevices.pivotLeftMotorID);
    protected final TalonFX pivotRightMotor = new TalonFX(CANDevices.pivotRightMotorID);
    protected final CANcoder pivotEncoder = new CANcoder(CANDevices.pivotEncoderID);
    protected final DigitalInput leftLimitSwitch = new DigitalInput(DIOPorts.pivotLeftLimitSwitchPort);
    protected final DigitalInput rightLimitSwitch = new DigitalInput(DIOPorts.pivotRightLimitSwitchPort);

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/PID/kP", 5);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/PID/kI", 0); 
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/PID/kD", 0);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/PID/Profile/kV", 5);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/PID/Profile/kA", 10);
    private final LoggedTunableNumber kJ = new LoggedTunableNumber("Pivot/PID/Profile/kJ", 0);

    private final LoggedTunableNumber ffkS = new LoggedTunableNumber("Pivot/FF/kS", 0);
    private final LoggedTunableNumber ffkG = new LoggedTunableNumber("Pivot/FF/kG", 0.03);
    private final LoggedTunableNumber ffkV = new LoggedTunableNumber("Pivot/FF/kV", 1.5);
    private final LoggedTunableNumber ffkA = new LoggedTunableNumber("Pivot/FF/kA", 0);
    
    public PivotIOFalcon() {
        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.RotorToSensorRatio = PivotConstants.motorToEncoderRatio.ratio();
        motorConfig.Feedback.SensorToMechanismRatio = PivotConstants.encoderToMechanismRatio.ratio();
        motorConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorConfig.Feedback.FeedbackRotorOffset = 0;
        /*
        motorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
        motorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        motorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;        
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
        */
        motorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
        motorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.Disabled;
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Pivot.ampAltitude.in(Rotations);
        pivotLeftMotor.getConfigurator().apply(motorConfig);
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotRightMotor.getConfigurator().apply(motorConfig);
        
        var encoderConfig = new CANcoderConfiguration();
        pivotEncoder.getConfigurator().refresh(encoderConfig);
        // encoderConfig.MagnetSensor.MagnetOffset = PivotConstants.pivotMagnetOffset;
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        pivotEncoder.getConfigurator().apply(encoderConfig);

        updateTunables();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            pivotLeftMotor.getPosition(),
            pivotLeftMotor.getVelocity(),
            pivotLeftMotor.getClosedLoopError()
        );

        pivotRightMotor.setControl(new StrictFollower(pivotLeftMotor.getDeviceID()));
    }

    private void updateTunables() {
        if(
            kP.hasChanged(hashCode()) |
            kI.hasChanged(hashCode()) |
            kD.hasChanged(hashCode()) |
            kV.hasChanged(hashCode()) |
            kA.hasChanged(hashCode()) |
            kJ.hasChanged(hashCode()) |
            ffkV.hasChanged(hashCode()) |
            ffkA.hasChanged(hashCode()) |
            ffkG.hasChanged(hashCode()) |
            ffkS.hasChanged(hashCode())
        ) {
            var pidConfig = new Slot0Configs();
            var profileConfig = new MotionMagicConfigs();
            pidConfig.kP = Units.rotationsToRadians(kP.get());
            pidConfig.kI = Units.rotationsToRadians(kI.get());
            pidConfig.kD = Units.rotationsToRadians(kD.get());
            profileConfig.MotionMagicCruiseVelocity= Units.radiansToRotations(kV.get());
            profileConfig.MotionMagicAcceleration = Units.radiansToRotations(kA.get());
            profileConfig.MotionMagicJerk = Units.radiansToRotations(kJ.get());
            pidConfig.kV = Units.rotationsToRadians(ffkV.get());
            pidConfig.kA = Units.rotationsToRadians(ffkA.get());
            pidConfig.kG = Units.rotationsToRadians(ffkG.get());
            pidConfig.kS = Units.rotationsToRadians(ffkS.get());
            pidConfig.GravityType = GravityTypeValue.Arm_Cosine;

            pivotLeftMotor.getConfigurator().apply(pidConfig);
            pivotLeftMotor.getConfigurator().apply(profileConfig);
        }
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotLeftMotor.updateFrom(pivotLeftMotor);
        inputs.pivotRightMotor.updateFrom(pivotRightMotor);
        inputs.pivotEncoder.updateFrom(pivotLeftMotor);

        updateTunables();

        var accel = Units.rotationsToRadians(pivotLeftMotor.getClosedLoopReferenceSlope().getValueAsDouble());
        var error = Units.rotationsToRadians(pivotLeftMotor.getClosedLoopError().getValueAsDouble());

        inputs.atGoal = 
            MathUtil.isNear(0, accel, 0.1) && 
            MathUtil.isNear(0, error, Pivot.tolerance.in(Radians))
        ;

        Logger.recordOutput("Pivot/Profile Position", Units.rotationsToRadians(pivotLeftMotor.getClosedLoopReference().getValueAsDouble()));
        Logger.recordOutput("Pivot/P Out", pivotLeftMotor.getClosedLoopProportionalOutput().getValueAsDouble());
        Logger.recordOutput("Pivot/FF Out", pivotLeftMotor.getClosedLoopFeedForward().getValueAsDouble());
    }

    @Override
    public void setPivotVoltage(double volts) {
        if(!(pivotRightMotor.getAppliedControl() instanceof StrictFollower)) {
            pivotRightMotor.setControl(new StrictFollower(pivotLeftMotor.getDeviceID()));
        }
        pivotLeftMotor.setVoltage(volts);
    }

    private final MotionMagicVoltage request = new MotionMagicVoltage(
        0,
        false,
        0,
        0,
        false,
        false,
        false
    );

    @Override
    public void setPivotPos(double pos) {
        if(!(pivotRightMotor.getAppliedControl() instanceof StrictFollower)) {
            pivotRightMotor.setControl(new StrictFollower(pivotLeftMotor.getDeviceID()));
        }
        pivotLeftMotor.setControl(request.withPosition(Units.radiansToRotations(pos)).withLimitReverseMotion(leftLimitSwitch.get() || rightLimitSwitch.get()));
    }

    @Override
    public void stop() {
        pivotLeftMotor.disable();
    }

    private static final ControlRequest COAST_OUT = new CoastOut();
    private static final ControlRequest NEUTRAL_OUT = new NeutralOut();
    @Override
    public void setCoast(boolean coast) {
        pivotLeftMotor.setControl(coast ? COAST_OUT : NEUTRAL_OUT);
        pivotRightMotor.setControl(coast ? COAST_OUT : NEUTRAL_OUT);
    }

    @Override
    public void enableSoftLimits(boolean enable) {
        var config = new SoftwareLimitSwitchConfigs();
        pivotLeftMotor.getConfigurator().refresh(config);
        if(config.ForwardSoftLimitEnable == enable && config.ReverseSoftLimitEnable == enable) return;
        config.ForwardSoftLimitEnable = enable;
        config.ReverseSoftLimitEnable = enable;
        pivotLeftMotor.getConfigurator().apply(config);
    }

    public void setRotorOffset(double rads) {
        var config = new FeedbackConfigs();
        pivotLeftMotor.getConfigurator().refresh(config);
        var rots = Units.radiansToRotations(rads);
        if(MathUtil.isNear(rots, config.FeedbackRotorOffset, 1e-3)) return;
        config.FeedbackRotorOffset = rots;
        pivotLeftMotor.getConfigurator().apply(config);
    }

    @Override
    public void zeroEncoder() {
        var config = new MagnetSensorConfigs();
        pivotEncoder.getConfigurator().refresh(config);
        config.MagnetOffset = pivotEncoder.getAbsolutePosition().getValueAsDouble();
        pivotEncoder.getConfigurator().apply(config);
    }
}
