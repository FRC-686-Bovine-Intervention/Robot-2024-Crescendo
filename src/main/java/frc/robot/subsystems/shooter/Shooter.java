// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
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
            }
        },
        SHOOTING(RobotState.getInstance().aimingParameters::targetShooterSpeed, RobotState.getInstance().aimingParameters::minimumShooterSpeed),
        PREEMPTIVE(new LoggedTunableNumber("Shooter/Target Speed/Pre-emptive", 17), () -> Double.POSITIVE_INFINITY),
        PASS(new LoggedTunableNumber("Shooter/Target Speed/Pass", 17)),
        SUPER_PASS(
            new LoggedTunableNumber("Shooter/Target Speed/Super Pass", 12),
            new LoggedTunableNumber("Shooter/Minimum Speed/Super Pass", 9),
            new LoggedTunableNumber("Shooter/Maximum Speed/Super Pass", 13)
        ),
        AMP(new LoggedTunableNumber("Shooter/Target Speed/Amp", 2)),
        CUSTOM(new LoggedTunableNumber("Shooter/Target Speed/Custom", 30)),
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
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/Average MPS", getAverageSurfaceSpeed());
        Logger.recordOutput("Shooter/Timer", followUpTimer.get());

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

    // private Command surfaceSpeed(DoubleSupplier mps) {
    //     return surfaceSpeed(mps, () -> mps.getAsDouble() - 1);
    // }

    // private Command surfaceSpeed(DoubleSupplier mps, DoubleSupplier acceptableMPS) {
    //     var subsystem = this;
    //     return new Command() {
    //         {
    //             addRequirements(subsystem);
    //             setName("Set Surface Speed");
    //         }
    //         @Override
    //         public void initialize() {
    //             execute();
    //         }
    //         @Override
    //         public void execute() {
    //             targetSpeed = mps.getAsDouble();
    //             shooterIO.setLeftSurfaceSpeed(targetSpeed);
    //             shooterIO.setRightSurfaceSpeed(targetSpeed);
    //             readyToShoot = getAverageSurfaceSpeed() >= acceptableMPS.getAsDouble() && getAverageSurfaceSpeed() <= targetSpeed + 2;
    //         }
    //         @Override
    //         public void end(boolean interrupted) {
    //             followUpTimer.stop();
    //             followUpTimer.reset();
    //             shooterIO.stop();
    //             readyToShoot = false;
    //         }
    //     };
    // }

    // private Command followUp(BooleanSupplier shot) {
    //     return new Command() {
    //         {
    //             setName("Wait for Followup");
    //         }
    //         @Override
    //         public void execute() {
    //             if(shot.getAsBoolean()) {
    //                 followUpTimer.start();
    //             }
    //         }
    //         @Override
    //         public void end(boolean interrupted) {
    //             followUpTimer.stop();
    //             followUpTimer.reset();
    //         }
    //         @Override
    //         public boolean isFinished() {
    //             return followUpTimer.hasElapsed(followUpTime.get());
    //         }
    //     };
    // }

    // private Command surfaceSpeedWithFinish(DoubleSupplier mps, BooleanSupplier shot) {
    //     return surfaceSpeedWithFinish(mps, () -> mps.getAsDouble() - 1, shot);
    // }
    // private Command surfaceSpeedWithFinish(DoubleSupplier mps, DoubleSupplier acceptableMPS, BooleanSupplier shot) {
    //     return followUp(shot).deadlineWith(surfaceSpeed(mps, acceptableMPS)).withName("Set Surface Speed Finish");
    // }

    // public Command shootWithTunableNumber() {
    //     return surfaceSpeed(tuningMPS::get).withName("Shoot with tunable number");
    // }

    // public Command shoot(Supplier<Translation2d> FORR) {
    //     return surfaceSpeed(() -> ShooterConstants.targetShooterSpeed.get(FORR.get().getNorm()), () -> ShooterConstants.minimumShooterSpeed.get(FORR.get().getNorm())).withName("Shoot at pos");
    // }

    // public Command shoot(Supplier<Translation2d> FORR, BooleanSupplier shot) {
    //     return surfaceSpeedWithFinish(() -> ShooterConstants.targetShooterSpeed.get(FORR.get().getNorm()), () -> ShooterConstants.minimumShooterSpeed.get(FORR.get().getNorm()), shot).withName("Shoot at pos");
    // }

    // public Command preemptiveSpinup() {
    //     return surfaceSpeed(preemtiveMPS::get).withName("Pre-emptive Spinup");
    // }

    // public Command amp() {
    //     return surfaceSpeed(ampMPS::get, () -> 500).withName("Amp");
    // }

    public Command setGoalCommand(Goal goal) {
        return startEnd(
            () -> this.goal = goal,
            () -> this.goal = Goal.IDLE
        )
        .withName("Shooter " + goal.name());
    }
}
