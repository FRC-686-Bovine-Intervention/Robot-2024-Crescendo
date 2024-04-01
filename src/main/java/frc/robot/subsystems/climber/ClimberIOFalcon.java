package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.LoggedTunableNumber;

public class ClimberIOFalcon implements ClimberIO {
    private final TalonFX climberMotor = new TalonFX(CANDevices.climberID);

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Climber/PID/kP", 40);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Climber/PID/kI", 0);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Climber/PID/kD", 0);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Climber/PID/Profile/kV", 0.38);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Climber/PID/Profile/kA", 0.4);
    private final LoggedTunableNumber kJ = new LoggedTunableNumber("Climber/PID/Profile/kJ", 0);
    private final LoggedTunableNumber ffkV = new LoggedTunableNumber("Climber/FF/kV", 35);
    private final LoggedTunableNumber ffkA = new LoggedTunableNumber("Climber/FF/kA", 0);
    private final LoggedTunableNumber ffkG = new LoggedTunableNumber("Climber/FF/kG", 0);
    private final LoggedTunableNumber ffkS = new LoggedTunableNumber("Climber/FF/kS", 0);

    public ClimberIOFalcon() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = ClimberConstants.motorToSurface.rotPerSurface();

        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = Climber.POS_ZERO;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Climber.POS_DEPLOY;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        

        climberMotor.getConfigurator().apply(config);

        updateTunables();
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
            pidConfig.kP = kP.get();
            pidConfig.kI = kI.get();
            pidConfig.kD = kD.get();
            profileConfig.MotionMagicCruiseVelocity= kV.get();
            profileConfig.MotionMagicAcceleration = kA.get();
            profileConfig.MotionMagicJerk = kJ.get();
            pidConfig.kV = ffkV.get();
            pidConfig.kA = ffkA.get();
            pidConfig.kG = ffkG.get();
            pidConfig.kS = ffkS.get();

            climberMotor.getConfigurator().apply(pidConfig);
            climberMotor.getConfigurator().apply(profileConfig);
        }
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberMotor.updateFrom(climberMotor);
        inputs.climberMotor.positionRad = climberMotor.getPosition().getValueAsDouble();

        updateTunables();

        Logger.recordOutput("Climber/Reverse Limit", climberMotor.getReverseLimit().getValue());
        Logger.recordOutput("Climber/Profile Position", climberMotor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("Climber/FF Out", climberMotor.getClosedLoopFeedForward().getValueAsDouble());
        Logger.recordOutput("Climber/P Out", climberMotor.getClosedLoopProportionalOutput().getValueAsDouble());
    }

    @Override
    public void setVoltage(double volts) {
        climberMotor.setVoltage(volts);
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
    public void setPosition(double pos) {
        climberMotor.setControl(request.withPosition(pos));
    }

    @Override
    public void stop() {
        climberMotor.disable();
    }
}
