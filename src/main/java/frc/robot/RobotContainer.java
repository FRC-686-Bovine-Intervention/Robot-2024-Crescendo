// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.ScoreNote;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.commands.AutoIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOFalcon550;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.commands.FeedForwardCharacterization;
import frc.robot.subsystems.drive.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.subsystems.drive.commands.FieldOrientedDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOFalcon550;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIONeo550;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOFalcon;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOFalcon;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIOPhotonVision;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.note.NoteVision;
import frc.robot.subsystems.vision.note.NoteVisionIOPhotonVision;
import frc.robot.subsystems.vision.note.NoteVisionIOSim;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LazyOptional;
import frc.robot.util.MathExtraUtil;
import frc.robot.util.controllers.ButtonBoard3x3;
import frc.robot.util.controllers.Joystick;
import frc.robot.util.controllers.XboxController;

public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Intake intake;
    private final Pivot pivot;
    @SuppressWarnings("unused")
    private final Kicker kicker;
    @SuppressWarnings("unused")
    private final Shooter shooter;
    @SuppressWarnings("unused")
    private final NoteVision noteVision;
    @SuppressWarnings("unused")
    private final ApriltagVision apriltagVision;
    @SuppressWarnings("unused")
    private final AutoIntake autoIntake;
    @SuppressWarnings("unused")
    private final Leds ledSystem;

    private final AutoSelector autoSelector = new AutoSelector("AutoSelector");

    private final Mechanism2d robotSideProfile = new Mechanism2d(3, 2, new Color8Bit(Color.kBlack));

    // Controller
    private final XboxController driveController = new XboxController(0);
    private final Joystick driveJoystick;
    @SuppressWarnings("unused")
    private final ButtonBoard3x3 buttonBoard = new ButtonBoard3x3(1);
    private final CommandJoystick simJoystick = new CommandJoystick(2);

    public RobotContainer() {
        System.out.println("[Init RobotContainer] Creating " + RobotType.getMode().name() + " " + RobotType.getRobot().name());
        switch(RobotType.getMode()) {
            case REAL:
                drive = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOFalcon550(DriveModulePosition.FRONT_LEFT),
                    new ModuleIOFalcon550(DriveModulePosition.FRONT_RIGHT),
                    new ModuleIOFalcon550(DriveModulePosition.BACK_LEFT),
                    new ModuleIOFalcon550(DriveModulePosition.BACK_RIGHT)
                );
                intake = new Intake(new IntakeIOFalcon550());
                kicker = new Kicker(new KickerIONeo550());
                shooter = new Shooter(new ShooterIOFalcon());
                pivot = new Pivot(new PivotIOFalcon());
                noteVision = new NoteVision(new NoteVisionIOPhotonVision(Camera.NoteVision.withRobotToIntermediate(pivot::getRobotToPivot)));
                apriltagVision = new ApriltagVision(Camera.AprilTagVision.toApriltagCamera(ApriltagCameraIOPhotonVision::new));
                autoIntake = new AutoIntake(noteVision::getTrackedNotes, noteVision::forgetNote);
                ledSystem = null;
                // ledSystem = new Leds(
                //     () -> drive.getCurrentCommand() != null && drive.getCurrentCommand() != drive.getDefaultCommand()
                // );
            break;
            case SIM:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                );
                intake = new Intake(new IntakeIOSim(simJoystick.button(1), simJoystick.button(2)));
                pivot = new Pivot(new PivotIOSim());
                kicker = new Kicker(new KickerIOSim(simJoystick.button(3)));
                shooter = new Shooter(new ShooterIOSim());
                noteVision = new NoteVision(new NoteVisionIOSim());
                apriltagVision = new ApriltagVision(Camera.AprilTagVision.toApriltagCamera());
                autoIntake = new AutoIntake(noteVision::getTrackedNotes, noteVision::forgetNote);
                ledSystem = null;
            break;
            default:
            case REPLAY:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {}
                );
                intake = new Intake(new IntakeIO() {});
                pivot = new Pivot(new PivotIO() {});
                kicker = new Kicker(new KickerIO() {});
                shooter = new Shooter(new ShooterIO() {});
                noteVision = null;
                apriltagVision = new ApriltagVision(Camera.AprilTagVision.toApriltagCamera());
                autoIntake = null;
                ledSystem = null;
            break;
        }

        driveJoystick = driveController.leftStick
            .smoothRadialDeadband(DriveConstants.driveJoystickDeadbandPercent)
            .radialSensitivity(0.75)
            .radialSlewRateLimit(DriveConstants.joystickSlewRateLimit);
        
        driveCustomFlick = FieldOrientedDrive.headingFromJoystick(
            driveController.rightStick.smoothRadialDeadband(0.85),
            () -> {
                var climbingMode = driveController.rightBumper().getAsBoolean();
                if(climbingMode) {
                    return new Rotation2d[]{
                        // Center Stage
                        Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(180))),
                        // Up Stage
                        Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(300))),
                        // Down Stage
                        Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(60))),
                    };
                }
                return new Rotation2d[]{
                    // Cardinals
                    Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(0))),
                    Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(90))),
                    Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(180))),
                    Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(270))),
                    // Subwoofer
                    Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(120))),
                    Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(240))),
                    // Source
                    Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(300))),
                };
            },
            () -> (kicker.hasNote() ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180))
        );

        System.out.println("[Init RobotContainer] Configuring Default Subsystem Commands");
        configureSubsystems();

        System.out.println("[Init RobotContainer] Configuring Button Bindings");
        configureButtonBindings();

        System.out.println("[Init RobotContainer] Configuring Autonomous Modes");
        configureAutos();

        if (Constants.tuningMode) {
            new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
        }
    }

    private void configureButtonBindings() {
        driveController.a()/* .and(() -> !(intake.hasNote() || kicker.hasNote())) */.whileTrue(intake.intake(drive::getChassisSpeeds));
        driveController.b().and(() -> drive.getChassisSpeeds().vxMetersPerSecond * (intake.getIntakeReversed() ? 1 : -1) >= 0.5).whileTrue(intake.outtake().alongWith(kicker.outtake()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        // driveController.povUp().whileTrue(pivot.movePivotManually(1));
        // driveController.povDown().whileTrue(pivot.movePivotManually(-1));
        driveController.leftStickButton().onTrue(Commands.runOnce(() -> drive.setPose(FieldConstants.speakerFront))/* new WheelCalibration(drive) */);
        driveController.povLeft().or(driveController.povDown()).onTrue(pivot.gotoZero());
        driveController.povUp().onTrue(pivot.gotoAmp());
        Supplier<Translation2d> shootAtPos = () -> {
            var speakerTrans = AllianceFlipUtil.apply(FieldConstants.speakerCenter);
            var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(drive.getChassisSpeeds(), drive.getPose().getRotation());
            var robotToSpeaker = speakerTrans.minus(drive.getPose().getTranslation());
            var robotToSpeakerNorm = robotToSpeaker.div(robotToSpeaker.getNorm());
            var velocityTowardsSpeaker = MathExtraUtil.dotProduct(robotToSpeakerNorm, new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
            var timeToSpeaker = drive.getPose().getTranslation().getDistance(speakerTrans) / (ShooterConstants.exitVelocity + velocityTowardsSpeaker);
            var chassisOffset = chassisSpeeds.times(timeToSpeaker);
            var translationalOffset = new Translation2d(chassisOffset.vxMetersPerSecond, chassisOffset.vyMetersPerSecond);
            var pointTo = speakerTrans.minus(translationalOffset);
            Logger.recordOutput("Shooter/Shoot At", pointTo);
            return pointTo;
        };
        driveController.rightBumper().toggleOnTrue(
            shooter.shoot(shootAtPos).asProxy().deadlineWith(
                new FieldOrientedDrive(
                    drive,
                    FieldOrientedDrive.joystickSpectatorToFieldRelative(
                        driveJoystick,
                        driveController.leftBumper()
                    ),
                    FieldOrientedDrive.pidControlledHeading(
                        FieldOrientedDrive.pointTo(
                            () -> Optional.of(shootAtPos.get()),
                            // () -> Optional.empty(),
                            () -> Rotation2d.fromDegrees(0)
                        ).orElse(driveCustomFlick)
                    )
                ),
                pivot.autoAim(shootAtPos).asProxy()
            )
            .withName("AutoAim")
        );
        driveController.leftTrigger.aboveThreshold(0.5).whileTrue(
            new FieldOrientedDrive(
                drive,
                autoIntake.getTranslationalSpeeds(
                    FieldOrientedDrive.joystickSpectatorToFieldRelative(
                        driveJoystick,
                        driveController.leftBumper()
                    )
                ),
                FieldOrientedDrive.pidControlledHeading(
                    FieldOrientedDrive.pointTo(autoIntake.targetLocation(), () -> Rotation2d.fromDegrees(180)).orElse(driveCustomFlick)
                )
            )
            .onlyWhile(() -> !intake.hasNote())
            .withName("AutoIntake")
        );
        driveController.rightTrigger.aboveThreshold(0.25).whileTrue(shooter.shootWith(() -> 90));
        driveController.x().whileTrue(kicker.kick());

        new Trigger(() -> 
            drive.getPose().getTranslation().getDistance(FieldConstants.speakerCenter) <= 3 && 
            kicker.hasNote() && 
            DriverStation.isEnabled()
        ).whileTrue(shooter.preemptiveSpinup().asProxy().onlyIf(() -> shooter.getCurrentCommand() == null));
        new Trigger(() -> pivot.atPos() && shooter.readyToShoot()).onTrue(kicker.kick().onlyWhile(() -> shooter.getCurrentCommand() != null));
        new Trigger(() -> driveController.leftStick.magnitude() > 0.1)
            .and(() -> {
                Command currentCommand = drive.getCurrentCommand();
                return currentCommand != null && currentCommand.getName().startsWith(Drive.autoDrivePrefix);
            })
            .onTrue(drive.getDefaultCommand());
    }

    private final LazyOptional<Rotation2d> driveCustomFlick;

    private void configureSubsystems() {
        drive.setDefaultCommand(
            new FieldOrientedDrive(
                drive,
                FieldOrientedDrive.joystickSpectatorToFieldRelative(
                    driveJoystick,
                    driveController.leftBumper()
                ),
                FieldOrientedDrive.pidControlledHeading(
                    driveCustomFlick
                )
            )
        );

        intake.setDefaultCommand(intake.doNothing());

        pivot.setDefaultCommand(pivot.gotoZero());

        new Trigger(pivot::readyToFeed).and(intake::noteReady).and(() -> !kicker.hasNote()).and(DriverStation::isEnabled).onTrue(SuperCommands.feedToKicker(intake, kicker));
    }

    private void configureAutos() {
        // autoSelector.addRoutine(new AutoRoutine(
        //     "Drive Characterization",
        //     new ArrayList<>(0),
        //     () -> new FeedForwardCharacterization(
        //         drive,
        //         true,
        //         new FeedForwardCharacterizationData("drive"),
        //         drive::runCharacterizationVolts,
        //         drive::getCharacterizationVelocity
        //     )
        // ));
        autoSelector.addRoutine(new ScoreNote(drive, shooter, pivot));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // AutoRoutine routine = autoChooser.get();
        // drive.setPose(routine.position.getPose());
        // return routine.command;
        return autoSelector.getSelectedAutoCommand();
    }

    public void robotPeriodic() {
        RobotState.getInstance().logOdometry();
        Camera.logCameraOverrides();
        Logger.recordOutput("Ready to shoot", pivot.atPos() && shooter.readyToShoot());
    }

    public void enabledInit() {
    }
}

