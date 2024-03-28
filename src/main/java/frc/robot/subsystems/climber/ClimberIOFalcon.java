package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANDevices;

public class ClimberIOFalcon implements ClimberIO {
    private final TalonFX climberMotor = new TalonFX(CANDevices.climberID);

    public ClimberIOFalcon() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = 2;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        climberMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberMotor.updateFrom(climberMotor);
    }

    @Override
    public void setVoltage(double volts) {
        climberMotor.setVoltage(volts);
    }
    @Override
    public void stop() {
        climberMotor.disable();
    }
}
