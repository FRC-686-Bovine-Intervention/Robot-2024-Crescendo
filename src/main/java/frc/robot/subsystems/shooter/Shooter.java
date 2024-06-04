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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MathExtraUtil;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private DoubleSupplier targetShootingSpeed, minimumShootingSpeed, maximumShootingSpeed;

    private void setShooterSpeedExtrimum(DoubleSupplier target, DoubleSupplier minimum, DoubleSupplier maximum) {
        targetShootingSpeed = target;
        minimumShootingSpeed = minimum;
        maximumShootingSpeed = maximum;
    }

    private void setShooterSpeedExtrimum(DoubleSupplier target, DoubleSupplier minimum) {
        setShooterSpeedExtrimum(target, minimum, () -> Double.POSITIVE_INFINITY);
    }

    private void setShooterSpeedExtrimum(DoubleSupplier target) {
        setShooterSpeedExtrimum(target, () -> target.getAsDouble() - 1);
    }

    public Command shooting() {
        var subsystem = this;
        return new Command() {
            {
                setName("Shooting");
                addRequirements(subsystem);
                setShooterSpeedExtrimum(
                    () -> RobotState.getInstance().aimingParameters.targetShooterSpeed(),
                    () -> RobotState.getInstance().aimingParameters.minimumShooterSpeed());
            }

            @Override
            public void execute() {
                var goalSpeed = targetShootingSpeed.getAsDouble() * ShooterConstants.shooterSpeedEnvCoef.getAsDouble();
                shooterIO.setLeftSurfaceSpeed(goalSpeed);
                shooterIO.setRightSurfaceSpeed(goalSpeed);
                Leds.getInstance().shooterBarGraph.set(true);
            }
        };
    }

    public Command preemptive() {
        var subsystem = this;
        return new Command() {
            private LoggedTunableNumber targetSpeed = new LoggedTunableNumber("Shooter/Pre-emptive/Target Speed", 17);

            {
                setName("Pre-emptive");
                addRequirements(subsystem);
                setShooterSpeedExtrimum(targetSpeed);
            }

            @Override
            public void execute() {
                var goalSpeed = targetShootingSpeed.getAsDouble() * ShooterConstants.shooterSpeedEnvCoef.getAsDouble();
                shooterIO.setLeftSurfaceSpeed(goalSpeed);
                shooterIO.setRightSurfaceSpeed(goalSpeed);
                Leds.getInstance().shooterBarGraph.set(false);
            }
        };
    }

    public Command pass() {
        var subsystem = this;
        return new Command() {
            private LoggedTunableNumber targetSpeed = new LoggedTunableNumber("Shooter/Pass/Target Speed", 17);

            {
                setName("Pass");
                addRequirements(subsystem);
                setShooterSpeedExtrimum(targetSpeed);
            }

            @Override
            public void execute() {
                var goalSpeed = targetShootingSpeed.getAsDouble() * ShooterConstants.shooterSpeedEnvCoef.getAsDouble();
                shooterIO.setLeftSurfaceSpeed(goalSpeed);
                shooterIO.setRightSurfaceSpeed(goalSpeed);
                Leds.getInstance().shooterBarGraph.set(true);
            }
        };
    }

    public Command superPass() {
        var subsystem = this;
        return new Command() {
            private final LoggedTunableNumber targetSpeed = new LoggedTunableNumber("Shooter/Super Pass/Target Speed", 12);
            private final LoggedTunableNumber minimumSpeed = new LoggedTunableNumber("Shooter/Super Pass/Minimum Speed", 9);
            private final LoggedTunableNumber maximumSpeed = new LoggedTunableNumber("Shooter/Super Pass/Maximum Speed", 13);

            {
                setName("Super Pass");
                addRequirements(subsystem);
                setShooterSpeedExtrimum(targetSpeed, minimumSpeed, maximumSpeed);
            }

            @Override
            public void execute() {
                var goalSpeed = targetShootingSpeed.getAsDouble() * ShooterConstants.shooterSpeedEnvCoef.getAsDouble();
                shooterIO.setLeftSurfaceSpeed(goalSpeed);
                shooterIO.setRightSurfaceSpeed(goalSpeed);
                Leds.getInstance().shooterBarGraph.set(true);
            }
        };
    }

    public Command amp() {
        var subsystem = this;
        return new Command() {
            private final LoggedTunableNumber targetSpeed = new LoggedTunableNumber("Shooter/Amp/Target Speed", 2);
            private final LoggedTunableNumber minimumSpeed = new LoggedTunableNumber("Shooter/Amp/Minimum Speed", 1.5);
            private final LoggedTunableNumber maximumSpeed = new LoggedTunableNumber("Shooter/Amp/Maximum Speed", 3);

            {
                setName("Amp");
                addRequirements(subsystem);
                setShooterSpeedExtrimum(targetSpeed, minimumSpeed, maximumSpeed);
            }

            @Override
            public void execute() {
                var goalSpeed = targetShootingSpeed.getAsDouble();
                shooterIO.setLeftSurfaceSpeed(goalSpeed);
                shooterIO.setRightSurfaceSpeed(goalSpeed);
                Leds.getInstance().shooterBarGraph.set(true);
            }
        };
    }

    public Command custom() {
        var subsystem = this;
        return new Command() {
            private final LoggedTunableNumber targetSpeed = new LoggedTunableNumber("Shooter/Custom/Target Speed", 10);
            private final LoggedTunableNumber minimumSpeed = new LoggedTunableNumber("Shooter/Custom/Minimum Speed", 50);
            private final LoggedTunableNumber maximumSpeed = new LoggedTunableNumber("Shooter/Custom/Maximum Speed", 50);

            {
                setName("Custom");
                addRequirements(subsystem);
                setShooterSpeedExtrimum(targetSpeed, minimumSpeed, maximumSpeed);
            }

            @Override
            public void execute() {
                var goalSpeed = targetShootingSpeed.getAsDouble() * ShooterConstants.shooterSpeedEnvCoef.getAsDouble();
                shooterIO.setLeftSurfaceSpeed(goalSpeed);
                shooterIO.setRightSurfaceSpeed(goalSpeed);
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
                setShooterSpeedExtrimum(() -> 0);
            }

            @Override
            public void execute() {}
        };
    }

    public Command idle() {
        var subsystem = this;
        return new Command() {
            {
                setName("Idle");
                addRequirements(subsystem);
                setShooterSpeedExtrimum(() -> 0);
            }

            @Override
            public void execute() {
                shooterIO.stop();
                Leds.getInstance().shooterBarGraph.set(false);
            }
        };
    }

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

        Leds.getInstance().shooterReady = readyToShoot();
        Leds.getInstance().shooterSpeed = getAverageSurfaceSpeed();
        Leds.getInstance().shooterTarget = targetShootingSpeed.getAsDouble();
    }

    public boolean readyToShoot() {
        return MathExtraUtil.isWithin(getAverageSurfaceSpeed(), minimumShootingSpeed.getAsDouble(), maximumShootingSpeed.getAsDouble());
    }

    public double getAverageSurfaceSpeed() {
        return MathExtraUtil.average(inputs.leftMotor.velocityRadPerSec, inputs.rightMotor.velocityRadPerSec);
    }
}
