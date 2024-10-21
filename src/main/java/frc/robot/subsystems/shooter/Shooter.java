// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.AimingParameters;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.Cooldown;
import frc.robot.util.LoggedInternalButton;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.MathExtraUtil;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private static final LoggedTunableMeasure<Velocity<Distance>> preemptiveTargetSpeed = new LoggedTunableMeasure<>("Shooter/Pre-emptive/Target Speed", MetersPerSecond.of(17));
    private static final LoggedTunableMeasure<Velocity<Distance>> passTargetSpeed = new LoggedTunableMeasure<>("Shooter/Pass/Target Speed", MetersPerSecond.of(17));
    private static final LoggedTunableMeasure<Velocity<Distance>> passTargetMinimum = new LoggedTunableMeasure<>("Shooter/Pass/Target Speed", MetersPerSecond.of(4));
    private static final LoggedTunableMeasure<Velocity<Distance>> passTargetMaximum = new LoggedTunableMeasure<>("Shooter/Pass/Target Speed", MetersPerSecond.of(21));
    private static final LoggedTunableMeasure<Velocity<Distance>> superPassTargetSpeed = new LoggedTunableMeasure<>("Shooter/Super Pass/Target Speed", MetersPerSecond.of(12));
    private static final LoggedTunableMeasure<Velocity<Distance>> superPassMinimumSpeed = new LoggedTunableMeasure<>("Shooter/Super Pass/Minimum Speed", MetersPerSecond.of(9));
    private static final LoggedTunableMeasure<Velocity<Distance>> superPassMaximumSpeed = new LoggedTunableMeasure<>("Shooter/Super Pass/Maximum Speed", MetersPerSecond.of(13));
    private static final LoggedTunableMeasure<Velocity<Distance>> ampTargetSpeed = new LoggedTunableMeasure<>("Shooter/Amp/Target Speed", MetersPerSecond.of(2));
    private static final LoggedTunableMeasure<Velocity<Distance>> ampMinimumSpeed = new LoggedTunableMeasure<>("Shooter/Amp/Minimum Speed", MetersPerSecond.of(1.5));
    private static final LoggedTunableMeasure<Velocity<Distance>> ampMaximumSpeed = new LoggedTunableMeasure<>("Shooter/Amp/Maximum Speed", MetersPerSecond.of(3));
    private static final LoggedTunableMeasure<Velocity<Distance>> customTargetSpeed = new LoggedTunableMeasure<>("Shooter/Custom/Target Speed", MetersPerSecond.of(15));
    private static final LoggedTunableMeasure<Velocity<Distance>> customMinimumSpeed = new LoggedTunableMeasure<>("Shooter/Custom/Minimum Speed", MetersPerSecond.of(50));
    private static final LoggedTunableMeasure<Velocity<Distance>> customMaximumSpeed = new LoggedTunableMeasure<>("Shooter/Custom/Maximum Speed", MetersPerSecond.of(50));
    private static final LoggedTunableMeasure<Velocity<Distance>> customIncrement = new LoggedTunableMeasure<>("Shooter/Custom/Increment", MetersPerSecond.of(0.5));

    public final InternalButton readyToShoot = new LoggedInternalButton("Shooter/Ready to Shoot");
    public final InternalButton autoShootEnabled = new LoggedInternalButton("Shooter/AutoShoot Enabled");
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
                },
                this
            )
        );

        SmartDashboard.putData("SysID/Shooter/Quasi Forward", routine.quasistatic(Direction.kForward).withName("SysID Quasistatic Forward"));
        SmartDashboard.putData("SysID/Shooter/Quasi Reverse", routine.quasistatic(Direction.kReverse).withName("SysID Quasistatic Reverse"));
        SmartDashboard.putData("SysID/Shooter/Dynamic Forward", routine.dynamic(Direction.kForward).withName("SysID Dynamic Forward"));
        SmartDashboard.putData("SysID/Shooter/Dynamic Reverse", routine.dynamic(Direction.kReverse).withName("SysID Dynamic Reverse"));
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Inputs/Shooter", inputs);
        Logger.recordOutput("Shooter/Average MPS", getAverageSurfaceSpeed());

        Leds.getInstance().shooterReady = readyToShoot.getAsBoolean();
        Leds.getInstance().shooterSpeed = getAverageSurfaceSpeed();
    }

    public double getAverageSurfaceSpeed() {
        return MathExtraUtil.average(inputs.leftMotor.velocityRadPerSec, inputs.rightMotor.velocityRadPerSec);
    }

    private void applyShooterSpeed(Supplier<Measure<Velocity<Distance>>> targetSpeed) {
        var goalSpeed = targetSpeed.get().in(MetersPerSecond) * ShooterConstants.shooterSpeedEnvCoef.getAsDouble();
        shooterIO.setLeftSurfaceSpeed(goalSpeed);
        shooterIO.setRightSurfaceSpeed(goalSpeed);
    }

    private void setReadyToShoot(Supplier<Measure<Velocity<Distance>>> minimum, Supplier<Measure<Velocity<Distance>>> maximum) {
        readyToShoot.setPressed(MathExtraUtil.isWithin(MetersPerSecond.of(getAverageSurfaceSpeed()), minimum.get(), maximum.get()));
    }

    private Command genCommand(
        String name,
        Supplier<Measure<Velocity<Distance>>> targetSpeed,
        Supplier<Measure<Velocity<Distance>>> minimumSpeed,
        Supplier<Measure<Velocity<Distance>>> maximumSpeed,
        boolean enableAutoShoot,
        boolean enableLEDs
    ) {
        var subsystem = this;
        return new Command() {
            {
                setName(name);
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                autoShootEnabled.setPressed(enableAutoShoot);
            }

            @Override
            public void execute() {
                applyShooterSpeed(targetSpeed);
                setReadyToShoot(minimumSpeed, maximumSpeed);
                Leds.getInstance().shooterTarget = targetSpeed.get().in(MetersPerSecond);
                Leds.getInstance().shooterBarGraph.set(enableLEDs);
            }

            @Override
            public void end(boolean interrupted) {
                readyToShoot.setPressed(false);
                autoShootEnabled.setPressed(false);
                Leds.getInstance().shooterBarGraph.set(false);
            }
        };
    }

    private static final Measure<Velocity<Distance>> VelocityMax = MetersPerSecond.of(Double.POSITIVE_INFINITY);
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
    public Command aimWithAutoShoot() {
        return genCommand(
            "Aim-AutoShoot",
            AimingParameters::targetShooterSpeed,
            AimingParameters::minimumShooterSpeed,
            () -> VelocityMax,
            true,
            true
        );
    }
    public Command aimWithoutAutoShoot() {
        return genCommand(
            "Aim",
            AimingParameters::targetShooterSpeed,
            AimingParameters::minimumShooterSpeed,
            () -> VelocityMax,
            false,
            true
        );
    }
    public Command preemptive() {
        return genCommand(
            "Pre-emptive",
            preemptiveTargetSpeed,
            () -> MetersPerSecond.zero(),
            () -> VelocityMax,
            false,
            false
        );
    }
    public Command pass() {
        return genCommand(
            "Pass",
            passTargetSpeed,
            passTargetMinimum,
            passTargetMaximum,
            false,
            true
        );
    }
    public Command superPass() {
        return genCommand(
            "Super Pass",
            superPassTargetSpeed,
            superPassMinimumSpeed,
            superPassMaximumSpeed,
            false,
            true
        );
    }
    public Command amp() {
        return genCommand(
            "Amp",
            ampTargetSpeed,
            ampMinimumSpeed,
            ampMaximumSpeed,
            false,
            true
        );
    }
    public Command custom() {
        return genCommand(
            "Custom",
            customTargetSpeed,
            customMinimumSpeed,
            customMaximumSpeed,
            false,
            true
        );
    }
    public Command customIncrement(BooleanSupplier increase, BooleanSupplier decrease) {
        Supplier<Measure<Velocity<Distance>>> speed = new Supplier<Measure<Velocity<Distance>>>() {
            private final MutableMeasure<Velocity<Distance>> speed = MutableMeasure.mutable(customTargetSpeed.get());
            private final Cooldown cooldown = new Cooldown();
            @Override
            public Measure<Velocity<Distance>> get() {
                Logger.recordOutput("Custom Shoot/Shooter Speed", speed);
                if(!cooldown.hasExpired()) {
                    return speed;
                }
                if(increase.getAsBoolean()) {
                    cooldown.reset(0.25);
                    speed.mut_acc(customIncrement.get());
                }
                if(decrease.getAsBoolean()) {
                    cooldown.reset(0.25);
                    speed.mut_minus(customIncrement.get());
                }

                return speed;
            }
        };
        return genCommand(
            "Custom Increment",
            speed,
            speed,
            speed,
            false,
            true
        );
    }
}
