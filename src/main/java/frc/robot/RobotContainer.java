// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.RobotType.Mode;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.ButtonAutoSelector;
import frc.robot.auto.ScoreBunny;
import frc.robot.auto.ScoreHighThenBunny;
import frc.robot.commands.DriveWithCustomFlick;
import frc.robot.commands.DriverAutoCommands;
import frc.robot.subsystems.arm.arm.Arm;
import frc.robot.subsystems.arm.arm.Arm.ArmPos;
import frc.robot.subsystems.arm.arm.ArmIO;
import frc.robot.subsystems.arm.arm.ArmIOFalcon;
import frc.robot.subsystems.arm.arm.ArmIOSim;
import frc.robot.subsystems.arm.manipulator.Manipulator;
import frc.robot.subsystems.arm.manipulator.ManipulatorIO;
import frc.robot.subsystems.arm.manipulator.ManipulatorIOSim;
import frc.robot.subsystems.arm.manipulator.ManipulatorIOTalon;
import frc.robot.subsystems.bunnyIntake.BunnyIntake;
import frc.robot.subsystems.bunnyIntake.BunnyIntake.BunnyPos;
import frc.robot.subsystems.bunnyIntake.BunnyIntakeIO;
import frc.robot.subsystems.bunnyIntake.BunnyIntakeIONeo;
import frc.robot.subsystems.bunnyIntake.BunnyIntakeIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIO550Falcon;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.LedData;
import frc.robot.subsystems.manualOverrides.ManualOverrides;
import frc.robot.subsystems.vision.AprilTagCamera;
import frc.robot.subsystems.vision.AprilTagCameraIO;
import frc.robot.subsystems.vision.AprilTagCameraIOPhotonVision;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.controllers.XboxController;

public class RobotContainer {
    // Subsystems
    private final Drive drive;
    @SuppressWarnings("unused")
    private final Leds ledSystem;

    private final AutoSelector autoSelector = new AutoSelector();

    private final Mechanism2d robotSideProfile = new Mechanism2d(3, 2, new Color8Bit(Color.kBlack));

    // Controller
    private final XboxController driveController = new XboxController(0);
    private final CommandJoystick buttonBoard = new CommandJoystick(1);

    public RobotContainer() {
        System.out.println("[Init RobotContainer] Creating " + RobotType.getRobot());
        switch(RobotType.getMode()) {
            case REAL:
                drive = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIO550Falcon(DriveModulePosition.FRONT_LEFT),
                    new ModuleIO550Falcon(DriveModulePosition.FRONT_RIGHT),
                    new ModuleIO550Falcon(DriveModulePosition.BACK_LEFT),
                    new ModuleIO550Falcon(DriveModulePosition.BACK_RIGHT)
                );
                ledSystem = new Leds(new LedData(
                    manip::hasBall,
                    manip::intaking,
                    () -> drive.getCurrentCommand() != null && drive.getCurrentCommand() != drive.getDefaultCommand(),
                    () -> manuOverrides.armOverridingBrake,
                    () -> manuOverrides.driveOverridingBreak,
                    autoSelector::getLEDColors
                ));
            break;
            case SIM:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                );
                vision = new Vision();
                manip = new Manipulator(new ManipulatorIOSim());
                bunnyIntake = new BunnyIntake(new BunnyIntakeIOSim());
                arm = new Arm(new ArmIOSim());
                manuOverrides = null;
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
                vision = new Vision();
                manip = new Manipulator(new ManipulatorIO() {});
                bunnyIntake = new BunnyIntake(new BunnyIntakeIO() {});
                arm = new Arm(new ArmIO() {});
                manuOverrides = null;
                ledSystem = null;
            break;
        }

        System.out.println("[Init RobotContainer] Configuring Button Bindings");
        configureButtonBindings();

        System.out.println("[Init RobotContainer] Configuring Default Subsystem Commands");
        configureSubsystems();

        System.out.println("[Init RobotContainer] Configuring Autonomous Modes");
        configureAutos();

        if (Constants.tuningMode) {
            new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
        }
    }

    private void configureButtonBindings() {

    }


    private void configureSubsystems() {
        drive.setDefaultCommand(
            new DriveWithCustomFlick(
                drive,
                driveController.leftStick
                    .smoothRadialDeadband(DriveConstants.driveJoystickDeadbandPercent)
                    .radialSensitivity(0.75)
                    .radialSlewRateLimit(DriveConstants.joystickSlewRateLimit)
                ,
                DriveWithCustomFlick.headingFromJoystick(
                    driveController.rightStick.smoothRadialDeadband(0.85),
                    () -> {
                        switch (arm.getTargetPos()) {
                            default:        return Rotation2d.fromDegrees(0);
                            case HighBack:
                            case LowBack:   return Rotation2d.fromDegrees(180);
                        }
                    }
                ),
                driveController.leftBumper()
            )
        );
    }

    private void configureAutos() {
        autoSelector.addRoutine(new ScoreHighThenBunny(drive, arm, manip, bunnyIntake));
        autoSelector.addRoutine(new ScoreBunny(drive, arm, bunnyIntake));
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
        Logger.recordOutput("Mechanism2d/Robot Side Profile", robotSideProfile);
    }

    public void enabledInit() {
        bunnyIntake.calibrate();
    }
}

