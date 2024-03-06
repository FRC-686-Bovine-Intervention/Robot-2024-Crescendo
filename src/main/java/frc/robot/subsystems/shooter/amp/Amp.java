package frc.robot.subsystems.shooter.amp;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;

public class Amp extends SubsystemBase {
    private final AmpIO ampIO;
    private final AmpIOInputsAutoLogged inputs = new AmpIOInputsAutoLogged();

    private static final double POS_ZERO = 0;
    private static final double POS_AMP = 0;

    private double calibVal = 0;

    private static final LoggedTunableNumber ampkP = new LoggedTunableNumber("Shooter/Amp/PID/kP", 0);
    private static final LoggedTunableNumber ampkI = new LoggedTunableNumber("Shooter/Amp/PID/kI", 0);
    private static final LoggedTunableNumber ampkD = new LoggedTunableNumber("Shooter/Amp/PID/kD", 0);
    private static final LoggedTunableNumber ampkV = new LoggedTunableNumber("Shooter/Amp/PID/kV", 0);
    private static final LoggedTunableNumber ampkA = new LoggedTunableNumber("Shooter/Amp/PID/kA", 0);
    private final ProfiledPIDController ampPID = new ProfiledPIDController(
        ampkP.get(),
        ampkI.get(),
        ampkD.get(),
        new Constraints(
            ampkV.get(),
            ampkA.get()
        )
    );

    private static final LoggedTunableNumber ampFFkV = new LoggedTunableNumber("Shooter/Amp/FF/kV", 0);
    private static final LoggedTunableNumber ampFFkA = new LoggedTunableNumber("Shooter/Amp/FF/kA", 0);
    private static final LoggedTunableNumber ampFFkS = new LoggedTunableNumber("Shooter/Amp/FF/kS", 0);
    private static final LoggedTunableNumber ampFFkG = new LoggedTunableNumber("Shooter/Amp/FF/kG", 0);
    private ArmFeedforward ampFF = new ArmFeedforward(
        ampFFkV.get(),
        ampFFkA.get(),
        ampFFkS.get(),
        ampFFkG.get()
    );

    private void updateTunables() {
        if(ampkP.hasChanged(hashCode()) | ampkI.hasChanged(hashCode()) | ampkD.hasChanged(hashCode())) {
            ampPID.setPID(ampkP.get(), ampkI.get(), ampkD.get());
        }
        if(ampFFkV.hasChanged(hashCode()) | ampFFkA.hasChanged(hashCode()) | ampFFkS.hasChanged(hashCode()) | ampFFkG.hasChanged(hashCode())) {
            ampFF = new ArmFeedforward(ampFFkS.get(), ampFFkG.get(), ampFFkV.get(), ampFFkA.get());
        }
    }

    private final Timer spikeTimer = new Timer();
    private static final LoggedTunableNumber spikeThreshold = new LoggedTunableNumber("Shooter/Amp/Spike Threshold", 500);
    private static final LoggedTunableNumber spikeTime = new LoggedTunableNumber("Shooter/Amp/Spike Time", 0.5);

    private boolean calibrated = false;

    public Amp(AmpIO ampIO) {
        this.ampIO = ampIO;
        new Trigger(() -> !calibrated).and(DriverStation::isEnabled).and(() -> !DriverStation.isTest()).onTrue(calibrate());
    }

    @AutoLogOutput(key = "Shooter/Amp/Position Rads")
    private double getPos() {
        return inputs.ampMotor.positionRad - calibVal;
    }

    @Override
    public void periodic() {
        ampIO.updateInputs(inputs);
        Logger.processInputs("Amp", inputs);
        updateTunables();
        if(inputs.ampMotor.currentAmps > spikeThreshold.get()) {
            spikeTimer.start();
        } else {
            spikeTimer.stop();
            spikeTimer.reset();
        }
    }

    private void followPID(double output, State setpoint) {
        var ff = ampFF.calculate(setpoint.position, setpoint.velocity);
        ampIO.setVoltage(output + ff);
        Logger.recordOutput("Shooter/Amp/PID out", output);
        Logger.recordOutput("Shooter/Amp/Profile Position", setpoint.position);
        Logger.recordOutput("Shooter/Amp/Profile Velocity", setpoint.velocity);
        Logger.recordOutput("Shooter/Amp/FF out", ff);
    }

    public Command amp() {
        return new ProfiledPIDCommand(
            ampPID,
            this::getPos,
            POS_AMP,
            this::followPID,
            this
        );
    }
    public Command zero() {
        return new ProfiledPIDCommand(
            ampPID,
            this::getPos,
            POS_ZERO,
            this::followPID,
            this
        );
    }
    public Command calibrate() {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Calibrate");
            }
            @Override
            public void execute() {
                ampIO.setVoltage(-4);
            }
            @Override
            public void end(boolean interrupted) {
                if(interrupted) return;
                calibVal = inputs.ampMotor.positionRad;
                calibrated = true;
            }
            @Override
            public boolean isFinished() {
                return spikeTimer.hasElapsed(spikeTime.get());
            }
        };
    }
}
