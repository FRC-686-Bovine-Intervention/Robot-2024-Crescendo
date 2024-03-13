package frc.robot.subsystems.shooter.amp;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.CANDevices;

public class AmpIONeo550 implements AmpIO {
    private final CANSparkMax ampMotor = new CANSparkMax(CANDevices.shooterAmpID, MotorType.kBrushless);

    public AmpIONeo550() {
        ampMotor.setInverted(true);
        ampMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        ampMotor.burnFlash();
    }

    @Override
    public void updateInputs(AmpIOInputs inputs) {
        inputs.ampMotor.updateFrom(ampMotor);
    }

    @Override
    public void setVoltage(double volts) {
        ampMotor.setVoltage(volts);
    }
}
