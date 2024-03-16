package frc.robot.subsystems.shooter.amp;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.CANDevices;
import frc.robot.util.LoggedTunableNumber;

public class AmpIONeo550 implements AmpIO {
    private final CANSparkMax ampMotor = new CANSparkMax(CANDevices.shooterAmpID, MotorType.kBrushless);

    private static final LoggedTunableNumber stallLimit = new LoggedTunableNumber("Amp/Current Limits/Stall Limit", 2);
    private static final LoggedTunableNumber freeLimit = new LoggedTunableNumber("Amp/Current Limits/Free Limit", 40);
    private static final LoggedTunableNumber limitRPM = new LoggedTunableNumber("Amp/Current Limits/Limit RPM", 60);

    public AmpIONeo550() {
        ampMotor.setInverted(true);
        ampMotor.setIdleMode(IdleMode.kBrake);
        ampMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        updateTunables();
    }

    private void updateTunables() {
        if(
            stallLimit.hasChanged(hashCode()) | 
            freeLimit.hasChanged(hashCode()) | 
            limitRPM.hasChanged(hashCode())
        ) {
            ampMotor.setSmartCurrentLimit((int)stallLimit.get(), (int)freeLimit.get(), (int)limitRPM.get());
            ampMotor.burnFlash();
        }
    }

    @Override
    public void updateInputs(AmpIOInputs inputs) {
        inputs.ampMotor.updateFrom(ampMotor);
        updateTunables();
    }

    @Override
    public void setVoltage(double volts) {
        ampMotor.setVoltage(volts);
    }
}
