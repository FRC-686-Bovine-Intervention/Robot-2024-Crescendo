package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.LoggedTunableNumber;

public class Climber extends SubsystemBase {
    private final ClimberIO climberIO;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final LoggedTunableNumber windDownVoltage = new LoggedTunableNumber("Climber/Wind Down Voltage", 3);

    public static final double POS_ZERO = 0;
    public static final double POS_DEPLOY = 0.402373046875;

    public Climber(ClimberIO climberIO) {
        System.out.println("[Init Climber] Instantiating Climber");
        this.climberIO = climberIO;
        System.out.println("[Init Climber] Climber IO: " + this.climberIO.getClass().getSimpleName());
        SmartDashboard.putData("Subsystems/Climber", this);
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(inputs);
        Logger.processInputs("Inputs/Climber", inputs);
        
        Leds.getInstance().climberPos = getClimberPos();
    }

    public double getClimberPos() {
        return inputs.climberMotor.positionRad;
    }

    public Command windDown() {
        var subsystem = this;
        return new Command() {
            {
                setName("Wind Down");
                addRequirements(subsystem);
            }
            @Override
            public void execute() {
                climberIO.setVoltage(-windDownVoltage.get());
            }
            @Override
            public void end(boolean interrupted) {
                climberIO.stop();
            }
        };
    }

    public Command deploy() {
        var subsystem = this;
        return new Command() {
            {
                setName("Deploy");
                addRequirements(subsystem);
            }
            @Override
            public void execute() {
                climberIO.setPosition(POS_DEPLOY);
                Leds.getInstance().climbingMode.set(true);
            }
            @Override
            public void end(boolean interrupted) {
                climberIO.stop();
                Leds.getInstance().climbingMode.set(false);
            }
        };
    }

    public Command retract() {
        var subsystem = this;
        return new Command() {
            {
                setName("Retract");
                addRequirements(subsystem);
            }
            @Override
            public void execute() {
                climberIO.setPosition(POS_ZERO);
                Leds.getInstance().climbing.set(true);
            }
            @Override
            public void end(boolean interrupted) {
                climberIO.stop();
                Leds.getInstance().climbing.set(false);
            }
        };
    }
}
