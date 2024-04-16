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
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.RobotState.AimingParameters;
import frc.robot.auto.AutoCommons.AutoPaths;
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
import frc.robot.subsystems.drive.commands.FieldOrientedDrive;
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
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.MathExtraUtil;
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

    private final AutoSelector autoSelector = new AutoSelector("AutoSelector");

    // private final Mechanism2d robotSideProfile = new Mechanism2d(3, 2, new Color8Bit(Color.kBlack));

    // Controller
    private final XboxController driveController = new XboxController(0);
    private final Joystick driveJoystick;
    private final Supplier<ChassisSpeeds> joystickTranslational;
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
                        simJoystick.button(1),
                        simJoystick.button(3)
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
        manualOverrides = new ManualOverrides(pivot::setCoast);
        driveJoystick = driveController.leftStick
            .smoothRadialDeadband(DriveConstants.driveJoystickDeadbandPercent)
            .radialSensitivity(0.75)
            .radialSlewRateLimit(DriveConstants.joystickSlewRateLimit);

        joystickTranslational = FieldOrientedDrive.joystickSpectatorToFieldRelative(
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
            new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
        }
    }

    private void configureSubsystems() {
        drive.translationSubsystem.setDefaultCommand(
            drive.translationSubsystem.fieldRelative(joystickTranslational).withName("Driver Control Field Relative")
        );

        rollers.intake.setDefaultCommand(rollers.setIntakeGoalCommand(Intake.Goal.ANTI_DEADZONE));
        rollers.kicker.setDefaultCommand(rollers.setKickerGoalCommand(Kicker.Goal.ANTI_DEADZONE));

        new Trigger(rollers::noNote)
        .onTrue(rollers.setGoalCommand(Rollers.Goal.ANTI_DEADZONE));

        new Trigger(rollers::noteInIntake)
        .and(DriverStation::isEnabled)
        .whileTrue(rollers.setGoalCommand(Rollers.Goal.FEED));

        new Trigger(rollers::noteInKicker)
        .and(() -> rollers.kicker.getGoal() != Kicker.Goal.KICK)
        .onTrue(
            rollers.setGoalCommand(Rollers.Goal.IDLE)
        );

        shooter.setDefaultCommand(shooter.setGoalCommand(Shooter.Goal.IDLE));

        pivot.setDefaultCommand(pivot.setGoalCommand(Pivot.Goal.IDLE));

        climber.setDefaultCommand(climber.windDown());
    }

    private void configureControls() {
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
        driveController.a().and(() -> !rollers.noteInKicker()).whileTrue(rollers.setIntakeGoalCommand(Intake.Goal.INTAKE));
        driveController.b()
            .and(() -> Math.abs(drive.getRobotRelativeSpeeds().vxMetersPerSecond) >= 0.25)
            .whileTrue(
                rollers.setGoalCommand(Rollers.Goal.EJECT)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            )
        ;
        
        // Kicker
        driveController.x().whileTrue(rollers.setKickerGoalCommand(Kicker.Goal.KICK));

        // Amp
        driveController.y().toggleOnTrue(
            Commands.either(
                Commands.parallel(
                    pivot.setGoalCommand(Pivot.Goal.AMP),
                    shooter.setGoalCommand(Shooter.Goal.AMP),
                    drive.rotationalSubsystem.pidControlledHeading(() -> Optional.of(FieldConstants.amp.getRotation()))
                ).withName("Amp").asProxy(),
                Commands.parallel(
                    pivot.setGoalCommand(Pivot.Goal.AMP),
                    shooter.setGoalCommand(Shooter.Goal.AMP)
                ).withName("Amp").asProxy(),
                () -> DriverStation.getMatchType() != MatchType.None
            )
        );

        // Shooter
        driveController.rightTrigger.aboveThreshold(0.25).whileTrue(shooter.setGoalCommand(Shooter.Goal.PASS));

        // Auto Aim
        driveController.rightBumper().toggleOnTrue(
            Commands.parallel(
                Commands.run(() -> RobotState.getInstance().aimingParameters = AimingParameters.from(drive.getPose().getTranslation(), drive.getFieldRelativeSpeeds())),
                pivot.setGoalCommand(Pivot.Goal.AIM),
                shooter.setGoalCommand(Shooter.Goal.SHOOTING),
                drive.rotationalSubsystem.pidControlledHeading(() -> Optional.of(RobotState.getInstance().aimingParameters.drivePose().getRotation()))
            )
            .until(rollers::noNote)
            .withName("Auto Aim")
        );

        // Aim from Subwoofer
        driveController.leftBumper().toggleOnTrue(
            Commands.parallel(
                pivot.setGoalCommand(Pivot.Goal.AIM),
                shooter.setGoalCommand(Shooter.Goal.SHOOTING)
            )
            .until(rollers::noNote)
            .beforeStarting(() -> RobotState.getInstance().aimingParameters = AimingParameters.from(AllianceFlipUtil.apply(FieldConstants.subwooferFront.getTranslation())))
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

        // SmartDashboard.putData("Recal Pivot", pivot.recal());
        SmartDashboard.putData("Reset pos", Commands.runOnce(() -> drive.setPose(new Pose2d(AllianceFlipUtil.apply(FieldConstants.subwooferFront).getTranslation(), drive.getRotation()))));

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
        new Trigger(() -> 
            shooter.readyToShoot() && 
            pivot.readyToShoot() && 
            MathExtraUtil.isNear(
                RobotState.getInstance().aimingParameters.drivePose().getRotation(),
                drive.getRotation(),
                Units.degreesToRadians(3)
            ) && 
            DriverStation.isTeleopEnabled() &&
            !Optional.ofNullable(shooter.getCurrentCommand()).map((c) -> c.getName().contains("Subwoofer")).orElse(false)
        ).onTrue(rollers.setKickerGoalCommand(Kicker.Goal.KICK));
        
        // Cancel Auto Drive
        new Trigger(() -> driveController.leftStick.magnitude() > 0.1)
            .and(() -> drive.translationSubsystem.getCurrentCommand() != null && drive.translationSubsystem.getCurrentCommand().getName().startsWith(Drive.autoDrivePrefix))
            .onTrue(drive.translationSubsystem.getDefaultCommand())
        ;
    }

    private void configureNotifications() {
        // Intake Notification
        new Trigger(rollers::noteInIntake)
        .onTrue(Leds.getInstance().noteAcquired.setCommand().withTimeout(1))
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(driveController.rumble(RumbleType.kBothRumble, 0.4));
        
        // Human Player Notification
        driveController.leftStickButton().onTrue(Leds.getInstance().humanPlayerFlash.setCommand().withTimeout(1));
    }

    private void configureAutos() {
        AutoPaths.preload();
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
    }

    private void configureSystemCheck() {
        SmartDashboard.putData("System Check/Pivot/Zero", pivot.getDefaultCommand());
        SmartDashboard.putData("System Check/Pivot/Amp", pivot.setGoalCommand(Pivot.Goal.AMP));
        SmartDashboard.putData("System Check/Climber/Wind Down", climber.getDefaultCommand());
        SmartDashboard.putData("System Check/Climber/Deploy", climber.deploy());
        SmartDashboard.putData("System Check/Climber/Retract", climber.retract());
        SmartDashboard.putData("System Check/Intake/Intake", rollers.setIntakeGoalCommand(Intake.Goal.INTAKE));
        SmartDashboard.putData("System Check/Kicker/Kick", rollers.setKickerGoalCommand(Kicker.Goal.KICK));
        SmartDashboard.putData("System Check/Shooter/Amp", shooter.setGoalCommand(Shooter.Goal.AMP));
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

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoSelector.getSelectedAutoCommand();
    }

    private final Alert xboxConnect = new Alert("Xbox Controller (Port 0) not connected", AlertType.ERROR);
    private final Alert buttonBoardConnect = new Alert("Button Board (Port 1) not connected", AlertType.WARNING);

    public void robotPeriodic() {
        RobotState.getInstance().log();
        Logger.recordOutput("NoteVisualizer/Internal Note", NoteVisualizer.logInternal());
        Camera.logCameraOverrides();
        xboxConnect.set(!driveController.isConnected());
        buttonBoardConnect.set(!buttonBoard.isConnected());
        Logger.recordOutput("Ready to shoot", pivot.atPos() && shooter.readyToShoot());
    }

    public void enabledInit() {
    }
}

