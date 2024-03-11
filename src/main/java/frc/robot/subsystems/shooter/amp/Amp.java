package frc.robot.subsystems.shooter.amp;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Amp extends SubsystemBase {
    private final AmpIO ampIO;
    private final AmpIOInputsAutoLogged inputs = new AmpIOInputsAutoLogged();

    private static final double POS_ZERO = 0;
    private static final double POS_AMP = 0;

    private double calibVal = 0;

    private final Timer spikeTimer = new Timer();
    private static final LoggedTunableNumber spikeThreshold = new LoggedTunableNumber("Shooter/Amp/Spike Threshold", 500);
    private static final LoggedTunableNumber spikeTime = new LoggedTunableNumber("Shooter/Amp/Spike Time", 0.5);

    public Amp(AmpIO ampIO) {
        System.out.println("[Init Amp] Instantiating Amp");
        this.ampIO = ampIO;
        System.out.println("[Init Amp] Amp IO: " + this.ampIO.getClass().getSimpleName());
        SmartDashboard.putData("Subsystems/Amp", this);  
    }

    @AutoLogOutput(key = "Shooter/Amp/Position Rads")
    private double getPos() {
        return inputs.ampMotor.positionRad - calibVal;
    }

    @Override
    public void periodic() {
        ampIO.updateInputs(inputs);
        Logger.processInputs("Amp", inputs);
        if(inputs.ampMotor.currentAmps > spikeThreshold.get()) {
            spikeTimer.start();
        } else {
            spikeTimer.stop();
            spikeTimer.reset();
        }
    }
    public Command amp() {
        return new Command() {
            {
                setName("Amp");
            }
            @Override
            public void execute() {
                ampIO.setVoltage(4);
            }
            @Override
            public void end(boolean interrupted) {
                ampIO.setVoltage(0);
            }
        };
    }
    public Command zero() {
        return new Command() {
            {
                setName("Zero");
            }
            @Override
            public void execute() {
                ampIO.setVoltage(-4);
            }
            @Override
            public void end(boolean interrupted) {
                ampIO.setVoltage(0);
            }
        };
    }
}
