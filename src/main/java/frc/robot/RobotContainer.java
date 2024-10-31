// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoManager;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.MASpikeWiggle;
import frc.robot.auto.Rush6Note;
import frc.robot.auto.Source4Note;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOFalcon;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOFalcon550;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.manualOverrides.ManualOverrides;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOFalcon;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.rollers.RollerSensorsIO;
import frc.robot.subsystems.rollers.RollerSensorsIODIO;
import frc.robot.subsystems.rollers.RollerSensorsIOSim;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIO;
import frc.robot.subsystems.rollers.intake.IntakeIOFalcon550;
import frc.robot.subsystems.rollers.intake.IntakeIOSim;
import frc.robot.subsystems.rollers.kicker.Kicker;
import frc.robot.subsystems.rollers.kicker.KickerIO;
import frc.robot.subsystems.rollers.kicker.KickerIONeo550;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOFalcon;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIOPhotonVision;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.note.NoteVision;
import frc.robot.subsystems.vision.note.NoteVisionIO;
import frc.robot.subsystems.vision.note.NoteVisionIOPhotonVision;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.Environment;
import frc.robot.util.controllers.ButtonBoard3x3;
import frc.robot.util.controllers.Joystick;
import frc.robot.util.controllers.XboxController;

public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final Rollers rollers;
    public final Pivot pivot;
    public final Shooter shooter;
    public final Climber climber;
    public final NoteVision noteVision;
    public final ApriltagVision apriltagVision;
    public final ManualOverrides manualOverrides;

    // Controller
    private final XboxController driveController = new XboxController(0);
    private final Joystick driveJoystick;
    private final Supplier<ChassisSpeeds> joystickTranslational;
    private final ButtonBoard3x3 buttonBoard = new ButtonBoard3x3(1);
    @SuppressWarnings("unused")
    private final CommandJoystick simJoystick = new CommandJoystick(2);

    public RobotContainer() {
        System.out.println("[Init RobotContainer] Creating " + RobotType.getMode().name() + " " + RobotType.getRobot().name());
        switch(RobotType.getMode()) {
            case REAL:
                drive = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOFalcon550(DriveConstants.modules[0]),
                    new ModuleIOFalcon550(DriveConstants.modules[1]),
                    new ModuleIOFalcon550(DriveConstants.modules[2]),
                    new ModuleIOFalcon550(DriveConstants.modules[3])
                );
                rollers = new Rollers(
                    new Intake(new IntakeIOFalcon550(), drive::getRobotRelativeSpeeds),
                    new Kicker(new KickerIONeo550()),
                    new RollerSensorsIODIO()
                );
                shooter = new Shooter(new ShooterIOFalcon());
                climber = new Climber(new ClimberIOFalcon());
                pivot = new Pivot(new PivotIOFalcon(), buttonBoard.povUp(), buttonBoard.povDown());
                // pivot = new Pivot(new PivotIOFalcon(), ()->false,()->false);
                noteVision = new NoteVision(new NoteVisionIOPhotonVision(Camera.NoteVision));
                apriltagVision = new ApriltagVision(Camera.LeftApriltag.toApriltagCamera(ApriltagCameraIOPhotonVision::new), Camera.RightApriltag.toApriltagCamera(ApriltagCameraIOPhotonVision::new));
            break;
            case SIM:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                );
                rollers = new Rollers(
                    new Intake(new IntakeIOSim(), drive::getRobotRelativeSpeeds),
                    new Kicker(new KickerIONeo550()),
                    new RollerSensorsIOSim(
                        // simJoystick.button(1).onTrue(Commands.print("------------------YIPPEEE")),
                        // simJoystick.button(3)
                        driveController.povDown(),
                        driveController.povUp()
                    )
                );
                pivot = new Pivot(new PivotIOSim(), ()->false,()->false);
                shooter = new Shooter(new ShooterIOSim());
                climber = new Climber(new ClimberIO() {});
                noteVision = new NoteVision(new NoteVisionIO() {});
                apriltagVision = new ApriltagVision(Camera.LeftApriltag.toApriltagCamera(), Camera.RightApriltag.toApriltagCamera());
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
                rollers = new Rollers(
                    new Intake(new IntakeIO() {}, drive::getRobotRelativeSpeeds),
                    new Kicker(new KickerIO() {}),
                    new RollerSensorsIO() {}
                );
                pivot = new Pivot(new PivotIO() {}, ()->false,()->false);
                shooter = new Shooter(new ShooterIO() {});
                climber = new Climber(new ClimberIO() {});
                noteVision = new NoteVision(new NoteVisionIO() {});
                apriltagVision = new ApriltagVision(Camera.LeftApriltag.toApriltagCamera(), Camera.RightApriltag.toApriltagCamera());
            break;
        }
        manualOverrides = new ManualOverrides();
        manualOverrides.redButton.and(DriverStation::isDisabled).toggleOnTrue(pivot.coast().until(DriverStation::isEnabled).withName("Pivot Button Coast"));
        driveJoystick = driveController.leftStick
            .smoothRadialDeadband(DriveConstants.driveJoystickDeadbandPercent)
            .radialSensitivity(0.75)
            .radialSlewRateLimit(DriveConstants.joystickSlewRateLimit)
        ;

        joystickTranslational = Drive.Translational.joystickSpectatorToFieldRelative(
            driveJoystick,
            () -> false
            // driveController.leftBumper()
        );
        
        System.out.println("[Init RobotContainer] Configuring Default Subsystem Commands");
        configureSubsystems();

        System.out.println("[Init RobotContainer] Configuring Controls");
        configureControls();
        
        System.out.println("[Init RobotContainer] Configuring Notifications");
        configureNotifications();

        System.out.println("[Init RobotContainer] Configuring Autonomous Modes");
        configureAutos();

        System.out.println("[Init RobotContainer] Configuring System Check");
        configureSystemCheck();

        if (Constants.tuningMode) {
            new Alert("Tuning mode active", AlertType.INFO).set(true);
        }
    }

    private void configureSubsystems() {
        drive.translationSubsystem.setDefaultCommand(drive.translationSubsystem.fieldRelative(joystickTranslational).withName("Driver Control Field Relative"));

        rollers.intake.setDefaultCommand(rollers.intake.antiDeadzone());
        rollers.kicker.setDefaultCommand(rollers.kicker.antiDeadZone());

        new Trigger(rollers::noNote)
            .onTrue(rollers.antiDeadzone())
        ;

        new Trigger(rollers::noteInIntake)
            .and(DriverStation::isEnabled)
            .whileTrue(rollers.intake.feed())
            .and(rollers.isKicking().negate())
            .whileTrue(rollers.kicker.feed())
        ;

        new Trigger(rollers::noteInKicker)
            .and(DriverStation::isEnabled)
            .onTrue(rollers.intake.idle())
            .and(rollers.isKicking().negate())
            .onTrue(rollers.kicker.idle())
        ;

        shooter.setDefaultCommand(shooter.idle());

        pivot.setDefaultCommand(pivot.idle());

        climber.setDefaultCommand(climber.windDown());
    }

    private void configureControls() {
        DriverStation.silenceJoystickConnectionWarning(true);
        // Rotation
        new Trigger(() -> driveController.rightStick.magnitude() > 0.85 && drive.rotationalSubsystem.getCurrentCommand() == null).onTrue(
            Commands.either(
                drive.rotationalSubsystem.headingFromJoystick(
                    driveController.rightStick.smoothRadialDeadband(0.85),
                    new Rotation2d[]{
                        // Center Stage
                        Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(+150))),
                        Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(-150))),
                        // Up Stage
                        Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(-90))),
                        Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(-30))),
                        // Down Stage
                        Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(+90))),
                        Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(+30))),
                    },
                    () -> Rotation2d.fromDegrees(-90)
                )
                .withName("Climbing")
                .asProxy(),
                drive.rotationalSubsystem.headingFromJoystick(
                    driveController.rightStick.smoothRadialDeadband(0.85),
                    new Rotation2d[]{
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
                    },
                    () -> (rollers.noNote() ? RobotConstants.intakeForward : RobotConstants.shooterForward)
                )
                .withName("DriveCustomFlick")
                .asProxy(),
                () -> climber.getCurrentCommand() != climber.getDefaultCommand()
            )
        );
        driveController.rightStickButton().toggleOnTrue(
            drive.rotationalSubsystem.defenseSpin(
                driveController.rightStick
                .smoothRadialDeadband(0.1)
                .radialSensitivity(0.75)
            )
        );
        // driveController.leftStickButton().onTrue(Commands.runOnce(() -> drive.setPose(new Pose2d(16,8, drive.getRotation()))));

        // Intake
        driveController.a().and(() -> !rollers.noteInKicker()).whileTrue(rollers.intake.intake());
        driveController.b()
            .and(() -> Math.abs(drive.getRobotRelativeSpeeds().vxMetersPerSecond) >= 0.25)
            .whileTrue(
                rollers.eject().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            )
        ;
        
        // Kicker
        driveController.x().whileTrue(rollers.kicker.kick());

        // Amp
        driveController.y().toggleOnTrue(
            Commands.either(
                Commands.parallel(
                    pivot.amp(),
                    shooter.amp(),
                    drive.rotationalSubsystem.pidControlledHeading(() -> Optional.of(FieldConstants.amp.getOurs().getRotation()))
                ).withName("Amp").asProxy(),
                Commands.parallel(
                    pivot.amp(),
                    shooter.amp()
                ).withName("Amp").asProxy(),
                Environment::isCompetition
            )
        );

        // Shooter
        driveController.rightTrigger.aboveThreshold(0.25).whileTrue(shooter.pass());

        SmartDashboard.putData("Custom Shoot",
            Commands.parallel(
                pivot.customIncremented(driveController.povUp(), driveController.povDown()),
                shooter.customIncrement(driveController.povLeft(), driveController.povRight())
            )
            .withName("Custom Shoot")
        );
        // Auto Aim
        // driveController.rightBumper().toggleOnTrue(
        //     Commands.parallel(
        //         pivot.customIncremented(driveController.povUp(), driveController.povDown()),
        //         shooter.customIncrement(driveController.povLeft(), driveController.povRight())
        //     )
        //     .until(rollers::noNote)
        // );
        // driveController.rightBumper().toggleOnTrue(
        //     Commands.parallel(
        //         Commands.run(() -> AimingParameters.setFrom(drive)),
        //         pivot.aim(),
        //         shooter.aimWithAutoShoot(),
        //         drive.rotationalSubsystem.pidControlledHeading(() -> Optional.of(AimingParameters.shotPose().getRotation()))
        //     )
        //     .until(rollers::noNote)
        //     .withName("Auto Aim")
        // );
        driveController.rightBumper().toggleOnTrue(
            Commands.either(
                Commands.waitUntil(
                    () -> AimingParameters.withinAzimuthTolerance(drive.getPose())
                    && shooter.readyToAutoShoot.getAsBoolean()
                    && pivot.atPos.getAsBoolean()
                )
                .andThen(
                    rollers.kicker
                        .kick()
                        .until(rollers::noteExited)
                        .withName("Auto Kick")
                        .asProxy()
                )
                .deadlineWith(
                    Commands.parallel(
                        Commands.run(() -> AimingParameters.setFrom(drive)),
                        pivot.aim(),
                        shooter.aimWithAutoShoot(),
                        drive.rotationalSubsystem.pidControlledHeading(() -> Optional.of(AimingParameters.shotPose().getRotation()))
                    )
                    .withName("Auto Aim")
                    .asProxy()
                ),
                Commands.parallel(
                    Commands.run(() -> AimingParameters.setFrom(drive, FieldConstants.passAimPoint.getOurs())),
                    pivot.superPass(),
                    shooter.superPass(),
                    drive.rotationalSubsystem.pidControlledHeading(() -> Optional.of(AimingParameters.shotPose().getRotation()))
                )
                .until(rollers::noNote)
                .withName("Super Pass")
                .asProxy(),
                () -> RobotState.getInstance().getPose().getTranslation().getDistance(FieldConstants.passAimPoint.getOurs().toTranslation2d()) < 5.5
            )
            .onlyIf(rollers::noteInKicker)
        );

        // SmartDashboard.putData(
        //     "Super Pass",
        //     Commands.parallel(
        //         Commands.run(() -> AimingParameters.setFrom(drive, FieldConstants.passAimPoint)),
        //         pivot.superPass(),
        //         shooter.superPass(),
        //         drive.rotationalSubsystem.pidControlledHeading(() -> Optional.of(AimingParameters.shotPose().getRotation()))
        //     )
        //     .withName("Super Pass")
        // );

        // Aim from Subwoofer
        driveController.leftBumper().toggleOnTrue(
            Commands.parallel(
                Commands.run(() -> AimingParameters.setFrom(FieldConstants.subwooferFront.getOurs().getTranslation())),
                pivot.aim(),
                shooter.aimWithoutAutoShoot()
            )
            .until(rollers::noNote)
            .withName("Shoot From Subwoofer")
        );

        // Auto Intake
        driveController.leftTrigger.aboveThreshold(0.25)
            .and(noteVision::hasTarget)
            .whileTrue(
                noteVision.autoIntake(
                    noteVision.applyDotProduct(joystickTranslational),
                    rollers::noNote,
                    drive
                )
            )
        ;

        SmartDashboard.putData("Recal Pivot", pivot.recal());
        SmartDashboard.putData("Reset pos", Commands.runOnce(() -> drive.setPose(new Pose2d(FieldConstants.subwooferFront.getOurs().getTranslation(), drive.getRotation()))));
        
        // Auto Drive
        // driveController.povUp().onTrue(drive.driveToFlipped(FieldConstants.pathfindSource));
        // driveController.povDown().onTrue(drive.driveToFlipped(FieldConstants.pathfindSpeaker));
        // driveController.povLeft().or(driveController.povRight()).onTrue(drive.driveToFlipped(FieldConstants.amp));

        // Climber
        driveController.start().toggleOnTrue(climber.deploy());
        driveController.back().toggleOnTrue(climber.retract());

        // Pre-emptive Spinup
        // new Trigger(() -> 
        //     drive.getPose().getTranslation().getDistance(AllianceFlipUtil.apply(FieldConstants.speakerAimPoint)) <= 6 && 
        //     MathUtil.isNear(
        //         Math.atan2(
        //             AllianceFlipUtil.apply(FieldConstants.speakerAimPoint).getY() - drive.getPose().getY(),
        //             AllianceFlipUtil.apply(FieldConstants.speakerAimPoint).getX() - drive.getPose().getX()
        //         ),
        //         drive.getRotation().getRadians(),
        //         1
        //     ) &&
        //     kicker.hasNote() && 
        //     DriverStation.isTeleopEnabled()
        // ).whileTrue(shooter.preemptiveSpinup().asProxy().onlyIf(() -> shooter.getCurrentCommand() == null));
        
        // Auto Fire
        // new Trigger(
        //     () -> AimingParameters.withinAzimuthTolerance(drive.getPose())
        // )
        //     .and(shooter.readyToAutoShoot)
        //     .and(pivot.atPos)
        //     .and(DriverStation::isTeleopEnabled)
        //     .onTrue(rollers.kicker.kick().until(rollers::noteExited).withName("Auto Kick"))
        // ;
        
        // Cancel Auto Drive
        // new Trigger(() -> driveController.leftStick.magnitude() > 0.1)
        //     .and(
        //         () -> drive.translationSubsystem.getCurrentCommand() != null
        //         && drive.translationSubsystem.getCurrentCommand().getName().startsWith(Drive.autoDrivePrefix)
        //     )
        //     .onTrue(drive.translationSubsystem.getDefaultCommand())
        // ;
    }

    private void configureNotifications() {
        // Intake Notification
        new Trigger(rollers::noteInIntake)
            .onTrue(Leds.getInstance().noteAcquired.setCommand().withTimeout(1))
            .and(DriverStation::isTeleopEnabled)
            .whileTrue(driveController.rumble(RumbleType.kBothRumble, 0.4))
        ;
        
        // Human Player Notification
        driveController.leftStickButton().onTrue(Leds.getInstance().humanPlayerFlash.setCommand().withTimeout(1));
    }

    private void configureAutos() {
        AutoPaths.preload();
        var autoSelector = new AutoSelector("AutoSelector");
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
        autoSelector.addDefaultRoutine(new MASpikeWiggle(this));
        autoSelector.addRoutine(new Rush6Note(this));
        autoSelector.addRoutine(new Source4Note(this));

        new AutoManager(autoSelector);
    }

    private void configureSystemCheck() {
        SmartDashboard.putData("System Check/Pivot/Zero", pivot.getDefaultCommand());
        SmartDashboard.putData("System Check/Pivot/Amp", pivot.amp());
        SmartDashboard.putData("System Check/Climber/Wind Down", climber.getDefaultCommand());
        SmartDashboard.putData("System Check/Climber/Deploy", climber.deploy());
        SmartDashboard.putData("System Check/Climber/Retract", climber.retract());
        SmartDashboard.putData("System Check/Intake/Intake", rollers.intake.intake());
        SmartDashboard.putData("System Check/Kicker/Kick", rollers.kicker.kick());
        SmartDashboard.putData("System Check/Shooter/Amp", shooter.amp());
        SmartDashboard.putData("System Check/Drive/Spin", 
            new Command() {
                private final Drive.Rotational rotationalSubsystem = drive.rotationalSubsystem;
                private final Timer timer = new Timer();
                {
                    addRequirements(rotationalSubsystem);
                    setName("TEST Spin");
                }
                public void initialize() {
                    timer.restart();
                }
                public void execute() {
                    rotationalSubsystem.driveVelocity(Math.sin(timer.get()) * 3);
                }
                public void end(boolean interrupted) {
                    timer.stop();
                    rotationalSubsystem.stop();
                }
            }
        );
        SmartDashboard.putData("System Check/Drive/Circle", 
            new Command() {
                private final Drive.Translational translationSubsystem = drive.translationSubsystem;
                private final Timer timer = new Timer();
                {
                    addRequirements(translationSubsystem);
                    setName("TEST Circle");
                }
                public void initialize() {
                    timer.restart();
                }
                public void execute() {
                    translationSubsystem.driveVelocity(
                        new ChassisSpeeds(
                            Math.cos(timer.get()) * 3,
                            Math.sin(timer.get()) * 3,
                            0
                        )
                    );
                }
                public void end(boolean interrupted) {
                    timer.stop();
                    translationSubsystem.stop();
                }
            }
        );
    }

    private final Alert xboxConnect = new Alert("Xbox Controller (Port 0) not connected", AlertType.ERROR);
    private final Alert buttonBoardConnect = new Alert("Button Board (Port 1) not connected", AlertType.WARNING);

    public void robotPeriodic() {
        RobotState.getInstance().log();
        Logger.recordOutput("NoteVisualizer/Internal Note", NoteVisualizer.logInternal());
        Camera.logCameraOverrides();
        xboxConnect.set(!driveController.isConnected());
        buttonBoardConnect.set(!buttonBoard.isConnected());
    }

    public void enabledInit() {
    }
}

