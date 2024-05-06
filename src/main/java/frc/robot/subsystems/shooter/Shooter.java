// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MathExtraUtil;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public static enum Goal {
        IDLE(() -> 0, () -> Double.POSITIVE_INFINITY){
            @Override
            public void runGoal(ShooterIO shooterIO) {
                shooterIO.stop();
                Leds.getInstance().shooterBarGraph.set(false);
            }
        },
        SHOOTING(
            () -> RobotState.getInstance().aimingParameters.targetShooterSpeed(),
            () -> RobotState.getInstance().aimingParameters.minimumShooterSpeed()
        ),
        PREEMPTIVE(
            new LoggedTunableNumber("Shooter/Pre-emptive/Target Speed", 17),
            () -> Double.POSITIVE_INFINITY
        ){
            @Override
            public void runGoal(ShooterIO shooterIO) {
                super.runGoal(shooterIO);
                Leds.getInstance().shooterBarGraph.set(false);
            }
        },
        PASS(
            new LoggedTunableNumber("Shooter/Pass/Target Speed", 17)
        ),
        SUPER_PASS(
            new LoggedTunableNumber("Shooter/Super Pass/Target Speed", 12),
            new LoggedTunableNumber("Shooter/Super Pass/Minimum Speed", 9),
            new LoggedTunableNumber("Shooter/Super Pass/Maximum Speed", 13)
        ),
        AMP(
            new LoggedTunableNumber("Shooter/Amp/Target Speed", 2),
            new LoggedTunableNumber("Shooter/Amp/Minimum Speed", 1.5),
            new LoggedTunableNumber("Shooter/Amp/Maximum Speed", 3)
        ),
        CUSTOM(
            new LoggedTunableNumber("Shooter/Custom/Target Speed", 10),
            new LoggedTunableNumber("Shooter/Custom/Minimum Speed", 50),
            new LoggedTunableNumber("Shooter/Custom/Maximum Speed", 50)
        ),
        SYSID(
            () -> 0, () -> Double.POSITIVE_INFINITY
        ) {
            @Override
            public void runGoal(ShooterIO shooterIO) {
                
            }
        },
        ;
        private final DoubleSupplier targetShootingSpeed;
        private final DoubleSupplier minimumShootingSpeed;
        private final DoubleSupplier maximumShootingSpeed;
        Goal(DoubleSupplier targetShootingSpeed) {
            this(targetShootingSpeed, () -> targetShootingSpeed.getAsDouble() - 1);
        }
        Goal(DoubleSupplier targetShootingSpeed, DoubleSupplier minimumShootingSpeed) {
            this(targetShootingSpeed, minimumShootingSpeed, () -> Double.POSITIVE_INFINITY);
        }
        Goal(DoubleSupplier targetShootingSpeed, DoubleSupplier minimumShootingSpeed, DoubleSupplier maximumShootingSpeed) {
            this.targetShootingSpeed = targetShootingSpeed;
            this.minimumShootingSpeed = minimumShootingSpeed;
            this.maximumShootingSpeed = maximumShootingSpeed;
        }
        public double getTargetSpeed() {
            return targetShootingSpeed.getAsDouble();
        }
        public double getMinimumSpeed() {
            return minimumShootingSpeed.getAsDouble();
        }
        public double getMaximumSpeed() {
            return maximumShootingSpeed.getAsDouble();
        }
        public void runGoal(ShooterIO shooterIO) {
            var goalSpeed = getTargetSpeed();
            shooterIO.setLeftSurfaceSpeed(goalSpeed);
            shooterIO.setRightSurfaceSpeed(goalSpeed);
            Leds.getInstance().shooterBarGraph.set(true);
        }
    }

    @AutoLogOutput(key = "Shooter/Goal")
    private Goal goal = Goal.IDLE;

    private static final LoggedTunableNumber followUpTime = new LoggedTunableNumber("Shooter/Follow Up Time", 0.25);
    private final Timer followUpTimer = new Timer();

    public Shooter(ShooterIO shooterIO) {
        System.out.println("[Init Shooter] Instantiating Shooter");
        this.shooterIO = shooterIO;
        System.out.println("[Init Shooter] Shooter IO: " + this.shooterIO.getClass().getSimpleName());
        SmartDashboard.putData("Subsystems/Shooter", this);

        var routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    shooterIO.setLeftVoltage(volts.in(Units.Volts));
                    shooterIO.setLeftVoltage(volts.in(Units.Volts));
                },
                (log) -> {
                    log.motor("left")
                        .angularPosition(Units.Radians.of(inputs.leftMotor.positionRad))
                        .angularVelocity(Units.RadiansPerSecond.of(inputs.leftMotor.velocityRadPerSec))
                    ;
                    log.motor("right")
                        .angularPosition(Units.Radians.of(inputs.rightMotor.positionRad))
                        .angularVelocity(Units.RadiansPerSecond.of(inputs.rightMotor.velocityRadPerSec))
                    ;
                },
                this
            )
        );

        SmartDashboard.putData("SysID/Shooter/Quasi Forward", routine.quasistatic(Direction.kForward));
        SmartDashboard.putData("SysID/Shooter/Quasi Reverse", routine.quasistatic(Direction.kReverse));
        SmartDashboard.putData("SysID/Shooter/Dynamic Forward", routine.dynamic(Direction.kForward));
        SmartDashboard.putData("SysID/Shooter/Dynamic Reverse", routine.dynamic(Direction.kReverse));
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/Average MPS", getAverageSurfaceSpeed());
        Logger.recordOutput("Shooter/Timer", followUpTimer.get());

        Leds.getInstance().shooterReady = readyToShoot();
        Leds.getInstance().shooterSpeed = getAverageSurfaceSpeed();
        Leds.getInstance().shooterTarget = getTargetSpeed();

        goal.runGoal(shooterIO);
    }

    public boolean readyToShoot() {
        return MathExtraUtil.isWithin(getAverageSurfaceSpeed(), goal.getMinimumSpeed(), goal.getMaximumSpeed());
    }

    public double getAverageSurfaceSpeed() {
        return MathExtraUtil.average(inputs.leftMotor.velocityRadPerSec, inputs.rightMotor.velocityRadPerSec);
    }

    public double getTargetSpeed() {
        return goal.targetShootingSpeed.getAsDouble();
    }

    public Command setGoalCommand(Goal goal) {
        return startEnd(
            () -> this.goal = goal,
            () -> this.goal = Goal.IDLE
        )
        .withName("Shooter " + goal.name());
    }
}
