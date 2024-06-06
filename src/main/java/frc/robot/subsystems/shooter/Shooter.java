// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MathExtraUtil;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public final InternalButton readyToShoot = new InternalButton();
    public final InternalButton autoShootEnabled = new InternalButton();
    public final Trigger readyToAutoShoot = readyToShoot.and(autoShootEnabled);

    public Shooter(ShooterIO shooterIO) {
        System.out.println("[Init Shooter] Instantiating Shooter");
        this.shooterIO = shooterIO;
        System.out.println("[Init Shooter] Shooter IO: " + this.shooterIO.getClass().getSimpleName());
        SmartDashboard.putData("Subsystems/Shooter", this);

        var routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> {
                    Logger.recordOutput("SysID/Shooter/State", state.toString());
                }
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    shooterIO.setLeftVoltage(volts.in(Units.Volts));
                    shooterIO.setRightVoltage(volts.in(Units.Volts));
                },
                (log) -> {
                    Logger.recordOutput("SysID/Shooter/Left Position", inputs.leftMotor.positionRad);
                    Logger.recordOutput("SysID/Shooter/Right Position", inputs.rightMotor.positionRad);
                    Logger.recordOutput("SysID/Shooter/Left Velocity", inputs.leftMotor.velocityRadPerSec);
                    Logger.recordOutput("SysID/Shooter/Right Velocity", inputs.rightMotor.velocityRadPerSec);
                    Logger.recordOutput("SysID/Shooter/Left Voltage", inputs.leftMotor.appliedVolts);
                    Logger.recordOutput("SysID/Shooter/Right Voltage", inputs.rightMotor.appliedVolts);
                    // log.motor("left")
                    //     .angularPosition(Units.Radians.of(inputs.leftMotor.positionRad))
                    //     .angularVelocity(Units.RadiansPerSecond.of(inputs.leftMotor.velocityRadPerSec))
                    //     .voltage(Units.Volts.of(inputs.leftMotor.appliedVolts))
                    //     .current(Units.Amps.of(inputs.leftMotor.currentAmps))
                    // ;
                    // log.motor("right")
                    //     .angularPosition(Units.Radians.of(inputs.rightMotor.positionRad))
                    //     .angularVelocity(Units.RadiansPerSecond.of(inputs.rightMotor.velocityRadPerSec))
                    //     .voltage(Units.Volts.of(inputs.rightMotor.appliedVolts))
                    //     .current(Units.Amps.of(inputs.rightMotor.currentAmps))
                    // ;
                },
                this
            )
        );

        // SmartDashboard.putData("SysID/Shooter/Quasi Forward", routine.quasistatic(Direction.kForward).deadlineWith(setGoalCommand(Goal.SYSID)).withName("SysID Quasistatic Forward"));
        // SmartDashboard.putData("SysID/Shooter/Quasi Reverse", routine.quasistatic(Direction.kReverse).deadlineWith(setGoalCommand(Goal.SYSID)).withName("SysID Quasistatic Reverse"));
        // SmartDashboard.putData("SysID/Shooter/Dynamic Forward", routine.dynamic(Direction.kForward).deadlineWith(setGoalCommand(Goal.SYSID)).withName("SysID Dynamic Forward"));
        // SmartDashboard.putData("SysID/Shooter/Dynamic Reverse", routine.dynamic(Direction.kReverse).deadlineWith(setGoalCommand(Goal.SYSID)).withName("SysID Dynamic Reverse"));
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/Average MPS", getAverageSurfaceSpeed());

        Leds.getInstance().shooterReady = readyToShoot.getAsBoolean();
        Leds.getInstance().shooterSpeed = getAverageSurfaceSpeed();
    }

    public double getAverageSurfaceSpeed() {
        return MathExtraUtil.average(inputs.leftMotor.velocityRadPerSec, inputs.rightMotor.velocityRadPerSec);
    }

    private void applyShooterSpeed(DoubleSupplier targetSpeed) {
        var goalSpeed = targetSpeed.getAsDouble() * ShooterConstants.shooterSpeedEnvCoef.getAsDouble();
        shooterIO.setLeftSurfaceSpeed(goalSpeed);
        shooterIO.setRightSurfaceSpeed(goalSpeed);
    }

    private void setReadyToShoot(DoubleSupplier minimum, DoubleSupplier maximum) {
        readyToShoot.setPressed(MathExtraUtil.isWithin(getAverageSurfaceSpeed(), minimum.getAsDouble(), maximum.getAsDouble()));
    }

    private final DoubleSupplier shootingTargetSpeed = () -> RobotState.getInstance().aimingParameters.targetShooterSpeed();
    private final DoubleSupplier shootingMinimumSpeed = () -> RobotState.getInstance().aimingParameters.minimumShooterSpeed();
    private final DoubleSupplier shootingMaximumSpeed = () -> Double.POSITIVE_INFINITY;
    public Command shooting() {
        var subsystem = this;
        return new Command() {
            {
                setName("Shooting");
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                autoShootEnabled.setPressed(true);
            }

            @Override
            public void execute() {
                applyShooterSpeed(shootingTargetSpeed);
                setReadyToShoot(shootingMinimumSpeed, shootingMaximumSpeed);
                Leds.getInstance().shooterTarget = shootingTargetSpeed.getAsDouble();
                Leds.getInstance().shooterBarGraph.set(true);
            }

            @Override
            public void end(boolean interrupted) {
                readyToShoot.setPressed(false);
                autoShootEnabled.setPressed(false);
            }
        };
    }

    private static LoggedTunableNumber preemptiveTargetSpeed = new LoggedTunableNumber("Shooter/Pre-emptive/Target Speed", 17);
    public Command preemptive() {
        var subsystem = this;
        return new Command() {
            {
                setName("Pre-emptive");
                addRequirements(subsystem);
            }

            @Override
            public void execute() {
                applyShooterSpeed(preemptiveTargetSpeed);
                Leds.getInstance().shooterTarget = preemptiveTargetSpeed.getAsDouble();
                Leds.getInstance().shooterBarGraph.set(false);
            }
        };
    }

    private static LoggedTunableNumber passTargetSpeed = new LoggedTunableNumber("Shooter/Pass/Target Speed", 17);
    private static LoggedTunableNumber passTargetMinimum = new LoggedTunableNumber("Shooter/Pass/Target Speed", 17);
    private static LoggedTunableNumber passTargetMaximum = new LoggedTunableNumber("Shooter/Pass/Target Speed", 17);
    public Command pass() {
        var subsystem = this;
        return new Command() {
            {
                setName("Pass");
                addRequirements(subsystem);
            }

            @Override
            public void execute() {
                applyShooterSpeed(passTargetSpeed);
                setReadyToShoot(passTargetMinimum, passTargetMaximum);
                Leds.getInstance().shooterTarget = passTargetSpeed.getAsDouble();
                Leds.getInstance().shooterBarGraph.set(true);
            }
        };
    }

    private static final LoggedTunableNumber superPassTargetSpeed = new LoggedTunableNumber("Shooter/Super Pass/Target Speed", 12);
    private static final LoggedTunableNumber superPassMinimumSpeed = new LoggedTunableNumber("Shooter/Super Pass/Minimum Speed", 9);
    private static final LoggedTunableNumber superPassMaximumSpeed = new LoggedTunableNumber("Shooter/Super Pass/Maximum Speed", 13);
    public Command superPass() {
        var subsystem = this;
        return new Command() {
            {
                setName("Super Pass");
                addRequirements(subsystem);
            }

            @Override
            public void execute() {
                applyShooterSpeed(superPassTargetSpeed);
                setReadyToShoot(superPassMinimumSpeed, superPassMaximumSpeed);
                Leds.getInstance().shooterTarget = superPassTargetSpeed.getAsDouble();
                Leds.getInstance().shooterBarGraph.set(true);
            }
        };
    }

    private static final LoggedTunableNumber ampTargetSpeed = new LoggedTunableNumber("Shooter/Amp/Target Speed", 2);
    private static final LoggedTunableNumber ampMinimumSpeed = new LoggedTunableNumber("Shooter/Amp/Minimum Speed", 1.5);
    private static final LoggedTunableNumber ampMaximumSpeed = new LoggedTunableNumber("Shooter/Amp/Maximum Speed", 3);
    public Command amp() {
        var subsystem = this;
        return new Command() {
            {
                setName("Amp");
                addRequirements(subsystem);
            }

            @Override
            public void execute() {
                applyShooterSpeed(ampTargetSpeed);
                setReadyToShoot(ampMinimumSpeed, ampMaximumSpeed);
                Leds.getInstance().shooterTarget = ampTargetSpeed.getAsDouble();
                Leds.getInstance().shooterBarGraph.set(true);
            }
        };
    }

    private static final LoggedTunableNumber customTargetSpeed = new LoggedTunableNumber("Shooter/Custom/Target Speed", 10);
    private static final LoggedTunableNumber customMinimumSpeed = new LoggedTunableNumber("Shooter/Custom/Minimum Speed", 50);
    private static final LoggedTunableNumber customMaximumSpeed = new LoggedTunableNumber("Shooter/Custom/Maximum Speed", 50);
    public Command custom() {
        var subsystem = this;
        return new Command() {
            {
                setName("Custom");
                addRequirements(subsystem);
            }

            @Override
            public void execute() {
                applyShooterSpeed(customTargetSpeed);
                setReadyToShoot(customMinimumSpeed, customMaximumSpeed);
                Leds.getInstance().shooterTarget = customTargetSpeed.getAsDouble();
                Leds.getInstance().shooterBarGraph.set(true);
            }
        };
    }

    public Command sysId() {
        var subsystem = this;
        return new Command() {
            {
                setName("SysId");
                addRequirements(subsystem);
            }

            @Override
            public void execute() {
                Leds.getInstance().shooterTarget = 0;
                Leds.getInstance().shooterBarGraph.set(false);
            }
        };
    }

    public Command idle() {
        var subsystem = this;
        return new Command() {
            {
                setName("Idle");
                addRequirements(subsystem);
            }

            @Override
            public void execute() {
                shooterIO.stop();
                Leds.getInstance().shooterTarget = 0;
                Leds.getInstance().shooterBarGraph.set(false);
            }
        };
    }
}
