// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.commands.AutoIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOFalcon550;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.commands.DriveWithCustomFlick;
import frc.robot.subsystems.drive.commands.FeedForwardCharacterization;
import frc.robot.subsystems.drive.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
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
        
        driveCustomFlick = DriveWithCustomFlick.headingFromJoystick(
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
        driveController.b().and(() -> drive.getChassisSpeeds().vxMetersPerSecond * (intake.getIntakeReversed() ? 1 : -1) >= 0.5).whileTrue(intake.outtake());
        driveController.povUp().whileTrue(pivot.movePivotManually(1));
        driveController.povDown().whileTrue(pivot.movePivotManually(-1));
        driveController.povLeft().onTrue(pivot.gotoZero());
        driveController.povRight().onTrue(pivot.gotoAmp());
        driveController.rightBumper().toggleOnTrue(
            new DriveWithCustomFlick(
                drive,
                DriveWithCustomFlick.joystickControlledFieldRelative(
                    driveJoystick,
                    driveController.leftBumper()
                ),
                DriveWithCustomFlick.pidControlledHeading(
                    DriveWithCustomFlick.pointTo(
                        () -> {
                            var speakerTrans = AllianceFlipUtil.apply(FieldConstants.speakerCenter);
                            var timeScalar = drive.getPose().getTranslation().getDistance(speakerTrans) / 5;
                            var chassisOffset = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds().times(timeScalar), drive.getPose().getRotation());
                            var translationalOffset = new Translation2d(chassisOffset.vxMetersPerSecond, chassisOffset.vyMetersPerSecond);
                            return Optional.of(speakerTrans.minus(translationalOffset));
                        },
                        () -> Rotation2d.fromDegrees(0)
                    )
                )
            )
            // .alongWith(shooter.shoot())
            .withName("AutoAim")
        );
        driveController.leftTrigger.aboveThreshold(0.5).whileTrue(
            new DriveWithCustomFlick(
                drive,
                autoIntake.getTranslationalSpeeds(
                    DriveWithCustomFlick.joystickControlledFieldRelative(
                        driveJoystick,
                        driveController.leftBumper()
                    )
                ),
                DriveWithCustomFlick.pidControlledHeading(
                    DriveWithCustomFlick.pointTo(autoIntake.targetLocation(), () -> Rotation2d.fromDegrees(180)).orElse(driveCustomFlick)
                )
            )
            .onlyWhile(() -> !intake.hasNote())
            .withName("AutoIntake")
        );
        driveController.rightTrigger.aboveThreshold(0.25).whileTrue(shooter.shoot());
        driveController.x().whileTrue(kicker.kick());

        // driveController.x().onTrue(drive.driveTo(AllianceFlipUtil.apply(FieldConstants.speakerFront)));
        // driveController.y().onTrue(drive.driveTo(AllianceFlipUtil.apply(FieldConstants.ampFront)));
        // driveController.a().onTrue(drive.driveTo(AllianceFlipUtil.apply(FieldConstants.podiumFront)));
        // driveController.b().onTrue(drive.driveTo(AllianceFlipUtil.apply(FieldConstants.sourceFront)));

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
            new DriveWithCustomFlick(
                drive,
                DriveWithCustomFlick.joystickControlledFieldRelative(
                    driveJoystick,
                    driveController.leftBumper()
                ),
                DriveWithCustomFlick.pidControlledHeading(
                    driveCustomFlick
                )
            )
        );

        intake.setDefaultCommand(intake.doNothing());

        new Trigger(pivot::readyToFeed).and(intake::noteReady).and(() -> !kicker.hasNote()).and(DriverStation::isEnabled).onTrue(SuperCommands.feedToKicker(intake, kicker));
    }

    private void configureAutos() {
        autoSelector.addRoutine(new AutoRoutine(
            "Drive Characterization",
            new ArrayList<>(0),
            () -> new FeedForwardCharacterization(
                drive,
                true,
                new FeedForwardCharacterizationData("drive"),
                drive::runCharacterizationVolts,
                drive::getCharacterizationVelocity
            )
        ));
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
        Logger.recordOutput("Testbot", new Pose2d());
        Logger.recordOutput("Mechanism2d/Robot Side Profile", robotSideProfile);
    }

    public void enabledInit() {
    }
}

