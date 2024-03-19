// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.BabyAuto;
import frc.robot.auto.MASpikeWiggle;
import frc.robot.auto.Rush6Note;
import frc.robot.auto.Source4Note;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOFalcon550;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.commands.FieldOrientedDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOFalcon550;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIONeo550;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.manualOverrides.ManualOverrides;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOFalcon;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOFalcon;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.amp.Amp;
import frc.robot.subsystems.shooter.amp.AmpIO;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIOPhotonVision;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.note.NoteVision;
import frc.robot.subsystems.vision.note.NoteVisionIOPhotonVision;
import frc.robot.subsystems.vision.note.NoteVisionIOSim;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.MathExtraUtil;
import frc.robot.util.controllers.ButtonBoard3x3;
import frc.robot.util.controllers.Joystick;
import frc.robot.util.controllers.XboxController;

public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final Intake intake;
    public final Pivot pivot;
    public final Kicker kicker;
    public final Shooter shooter;
    public final Amp amp;
    public final NoteVision noteVision;
    public final ApriltagVision apriltagVision;
    public final ManualOverrides manualOverrides;
    public final Leds leds;

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
                intake = new Intake(new IntakeIOFalcon550());
                kicker = new Kicker(new KickerIONeo550());
                shooter = new Shooter(new ShooterIOFalcon());
                amp = new Amp(new AmpIO() {});
                pivot = new Pivot(new PivotIOFalcon(), buttonBoard.povUp(), buttonBoard.povDown());
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
                intake = new Intake(new IntakeIOSim(simJoystick.button(1)));
                pivot = new Pivot(new PivotIOSim(), ()->false,()->false);
                kicker = new Kicker(new KickerIOSim(simJoystick.button(3)));
                shooter = new Shooter(new ShooterIOSim());
                amp = new Amp(new AmpIO() {});
                noteVision = new NoteVision(new NoteVisionIOSim());
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
                intake = new Intake(new IntakeIO() {});
                pivot = new Pivot(new PivotIO() {}, ()->false,()->false);
                kicker = new Kicker(new KickerIO() {});
                shooter = new Shooter(new ShooterIO() {});
                amp = new Amp(new AmpIO() {});
                noteVision = null;
                apriltagVision = new ApriltagVision(Camera.LeftApriltag.toApriltagCamera(), Camera.RightApriltag.toApriltagCamera());
            break;
        }
        // ledSystem = new Leds(
        //     () -> false,
        //     () -> intake.getIntakeCommand().equals(Optional.of(IntakeCommand.INTAKE)),
        //     () -> intake.getIntakeReversed(),
        //     () -> intake.getIntakeCommand().equals(Optional.of(IntakeCommand.FEED_TO_KICKER)),
        //     () -> kicker.hasNote()
        //     // () -> drive.getCurrentCommand() != null && drive.getCurrentCommand().getName().startsWith(Drive.autoDrivePrefix),
        //     // () -> intake.getIntakeCommand().equals(Optional.of(IntakeCommand.INTAKE)),
        //     // () -> intake.getIntakeReversed(),
        //     // () -> intake.getIntakeCommand().equals(Optional.of(IntakeCommand.FEED_TO_KICKER)),
        //     // () -> kicker.hasNote()
        // );
        manualOverrides = new ManualOverrides(pivot::setCoast);
        leds = new Leds();
        driveJoystick = driveController.leftStick
            .smoothRadialDeadband(DriveConstants.driveJoystickDeadbandPercent)
            .radialSensitivity(0.75)
            .radialSlewRateLimit(DriveConstants.joystickSlewRateLimit);

        joystickTranslational = FieldOrientedDrive.joystickSpectatorToFieldRelative(
            driveJoystick,
            driveController.leftBumper()
        );
        
        // driveCustomFlick = Drive.Rotational.headingFromJoystick(
        //     driveController.rightStick.smoothRadialDeadband(0.85),
        //     () -> {
        //         var climbingMode = driveController.rightBumper().getAsBoolean();
        //         if(climbingMode) {
        //             return new Rotation2d[]{
        //                 // Center Stage
        //                 Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(180))),
        //                 // Up Stage
        //                 Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(300))),
        //                 // Down Stage
        //                 Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(60))),
        //             };
        //         }
        //         return new Rotation2d[]{
        //             // Cardinals
        //             Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(0))),
        //             Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(90))),
        //             Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(180))),
        //             Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(270))),
        //             // Subwoofer
        //             Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(120))),
        //             Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(240))),
        //             // Source
        //             Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(300))),
        //         };
        //     },
        //     () -> (kicker.hasNote() ? RobotConstants.shooterForward : RobotConstants.intakeForward)
        // );

        System.out.println("[Init RobotContainer] Configuring Default Subsystem Commands");
        configureSubsystems();

        System.out.println("[Init RobotContainer] Configuring Controls");
        configureControls();
        
        System.out.println("[Init RobotContainer] Configuring Notifications");
        configureNotifications();

        System.out.println("[Init RobotContainer] Configuring Autonomous Modes");
        configureAutos();

        if (Constants.tuningMode) {
            new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
        }
    }

    private void configureSubsystems() {
        drive.translationSubsystem.setDefaultCommand(
            drive.translationSubsystem.fieldRelative(joystickTranslational).withName("Driver Control Field Relative")
        );
        drive.rotationalSubsystem.setDefaultCommand(
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
                    // var climbingMode = driveController.rightBumper().getAsBoolean();
                    // if(climbingMode) {
                    //     return new Rotation2d[]{
                    //         // Center Stage
                    //         Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(180))),
                    //         // Up Stage
                    //         Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(300))),
                    //         // Down Stage
                    //         Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(60))),
                    //     };
                    // }
                },
                () -> (kicker.hasNote() ? RobotConstants.shooterForward : RobotConstants.intakeForward)
            )
            .withName("DriveCustomFlick")
        );

        intake.setDefaultCommand(intake.antiDeadzone());
        kicker.setDefaultCommand(kicker.antiDeadzone());
        new Trigger(kicker::hasNote).and(() -> Optional.ofNullable(kicker.getCurrentCommand()).map((c) -> c.getName().contains("|")).orElse(false)).and(DriverStation::isEnabled).whileTrue(intake.doNothing().asProxy().alongWith(kicker.doNothing().asProxy()));

        pivot.setDefaultCommand(pivot.gotoZero());

        new Trigger(intake::hasNote).and(() -> !kicker.hasNote() && kicker.getCurrentCommand() == kicker.getDefaultCommand()).and(DriverStation::isEnabled).onTrue(SuperCommands.feedToKicker(intake, kicker));
    }

    private void configureControls() {
        // Intake
        driveController.a().and(() -> !(intake.hasNote() || kicker.hasNote())).whileTrue(intake.intake(drive::getChassisSpeeds));
        driveController.b().and(() -> Math.abs(drive.getChassisSpeeds().vxMetersPerSecond) >= 0.5).whileTrue(intake.outtake(drive::getChassisSpeeds).alongWith(kicker.outtake()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        
        // Kicker
        driveController.x().whileTrue(kicker.kick());

        // Amp
        driveController.y().toggleOnTrue(
            pivot.gotoAmp().asProxy()
            .alongWith(
                shooter.amp().asProxy()
            )
        );

        // Shooter
        driveController.rightTrigger.aboveThreshold(0.25).whileTrue(shooter.shootWithTunableNumber());

        // Auto Aim
        driveController.rightBumper().toggleOnTrue(SuperCommands.autoAim(drive.rotationalSubsystem, shooter, kicker, pivot));
        // driveController.leftBumper().toggleOnTrue(pivot.gotoVariable(driveController.povDown(), driveController.povUp()));

        // Auto Intake
        driveController.leftTrigger.aboveThreshold(0.25).and(noteVision::hasTarget).whileTrue(noteVision.autoIntake(noteVision.applyDotProduct(joystickTranslational), drive, intake));

        // Auto Drive
        driveController.povUp().onTrue(drive.driveToFlipped(FieldConstants.pathfindSource));
        driveController.povDown().onTrue(drive.driveToFlipped(FieldConstants.pathfindSpeaker));
        driveController.povLeft().or(driveController.povRight()).onTrue(drive.driveToFlipped(FieldConstants.amp));
        // driveController.rightStickButton().onTrue(Commands.runOnce(() -> drive.setPose(FieldConstants.subwooferFront)));

        // Defense Spin
        driveController.rightStickButton().toggleOnTrue(drive.rotationalSubsystem.spin(driveController.rightStick.x().multiply(DriveConstants.maxTurnRateRadiansPerSec)));

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
            SuperCommands.readyToShoot(shooter, pivot) && 
            MathExtraUtil.isNear(
                autoAimRotation(),
                drive.getRotation(),
                Units.degreesToRadians(3)
            ) && 
            DriverStation.isTeleopEnabled()
        ).onTrue(kicker.kick().asProxy().until(() -> shooter.getCurrentCommand() == null));
        
        // Cancel Auto Drive
        new Trigger(() -> driveController.leftStick.magnitude() > 0.1)
            .and(() -> drive.translationSubsystem.getCurrentCommand() != null && drive.translationSubsystem.getCurrentCommand().getName().startsWith(Drive.autoDrivePrefix))
            .onTrue(drive.translationSubsystem.getDefaultCommand())
        ;
    }
    private Rotation2d autoAimRotation() {
        var FORR = SuperCommands.autoAimFORR(drive);
        return new Rotation2d(FORR.get().getX(), FORR.get().getY());
    }

    private void configureNotifications() {
        // Intake Notification
        new Trigger(
            () -> intake.getIntakeCommand().equals(Optional.of(IntakeCommand.FEED_TO_KICKER))
        ).onTrue(
            leds.noteAcquired()
            .alongWith(
                driveController.rumble(RumbleType.kBothRumble, 0.4)
                .withTimeout(0.5)
            )
        );
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
        // autoSelector.addRoutine(new SpikeMarkShots(this));
        // autoSelector.addRoutine(new SpikeMarkAndCenterLine(this));
        // autoSelector.addRoutine(new CleanSpikes(this));
        autoSelector.addRoutine(new Rush6Note(this));
        autoSelector.addRoutine(new BabyAuto(this));
        autoSelector.addRoutine(new Source4Note(this));
        autoSelector.addDefaultRoutine(new MASpikeWiggle(this));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
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

