// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;
import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.apriltag.ApriltagCamera;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIO;
import frc.robot.util.GearRatio;
import frc.robot.util.swerve.ModuleLimits;

public final class Constants {

    public static enum Mode {
        REAL, SIM, REPLAY
    }

    public static final Mode simulationMode = Mode.SIM;
    public static final boolean tuningMode = true;

    public static final double dtSeconds = 0.02;
    public static final double loopFrequencyHz = 1.0/dtSeconds;

    public static Mode getMode() {
        if(Robot.isReal()) return Mode.REAL;
        return (simulationMode == Mode.REAL ? Mode.SIM : simulationMode);
    }

    public static final class CANDevices {

        // Drive
        public static final String driveCanBusName = "rio";
        // | Front Left
        public static final int frontLeftDriveMotorID  = 1;
        public static final int frontLeftTurnMotorID   = 1;
        // | Front Right
        public static final int frontRightDriveMotorID  = 2;
        public static final int frontRightTurnMotorID   = 2;
        // | Back Left
        public static final int backLeftDriveMotorID  = 3;
        public static final int backLeftTurnMotorID   = 3;
        // | Back Right
        public static final int backRightDriveMotorID  = 4;
        public static final int backRightTurnMotorID   = 4;

        // Intake
        public static final int intakeBeltMotorID   = 5;
        public static final int intakeRollerMotorID = 5;

        // Pivot
        public static final int pivotMotorID = 6;
        public static final int pivotEncoderID = 6;

        // Kicker
        public static final int kickerLeftID = 7;
        public static final int kickerRightID = 8;

        // Shooter
        public static final int shooterLeftID = 7;
        public static final int shooterRightID = 8;

        // Misc
        public static final int pigeonCanID = 0;
        public static final int candleCanID = 0;

        public static final double minCanUpdateRate = 4.0;
    }

    public static final class DIOPorts {
        // HID
        public static final int brakeSwitchPort = 9;
        public static final int ledSwitchPort = 8;

        // Intake
        public static final int intakeBottomSensorPort = 0;
        public static final int intakeTopSensorPort = 1;

        // Kicker
        public static final int kickerSensorPort = 2;
    }

    public static final class DriveConstants {
        public static int numDriveModules = 4;
        public static enum DriveModulePosition {
            FRONT_LEFT  (CANDevices.frontLeftDriveMotorID, CANDevices.frontLeftTurnMotorID, InvertedValue.CounterClockwise_Positive,
            0.75,
            new Translation2d( DriveConstants.trackWidthXMeters / 2.0,  DriveConstants.trackWidthYMeters / 2.0)),
            FRONT_RIGHT (CANDevices.frontRightDriveMotorID, CANDevices.frontRightTurnMotorID, InvertedValue.Clockwise_Positive,
            0.5,
            new Translation2d( DriveConstants.trackWidthXMeters / 2.0, -DriveConstants.trackWidthYMeters / 2.0)),
            BACK_LEFT   (CANDevices.backLeftDriveMotorID, CANDevices.backLeftTurnMotorID, InvertedValue.CounterClockwise_Positive,
            0.5,
            new Translation2d(-DriveConstants.trackWidthXMeters / 2.0,  DriveConstants.trackWidthYMeters / 2.0)),
            BACK_RIGHT  (CANDevices.backRightDriveMotorID, CANDevices.backRightTurnMotorID, InvertedValue.Clockwise_Positive,
            0.75,
            new Translation2d(-DriveConstants.trackWidthXMeters / 2.0, -DriveConstants.trackWidthYMeters / 2.0));
            public final int driveMotorID;
            public final int turnMotorID;
            // motor direction to drive 'forward' (cancoders at angles given in cancoderOffsetRotations)
            public final InvertedValue driveInverted;
            // absolute position of cancoder when drive wheel is facing 'forward'
            public final double cancoderOffsetRotations;
            public final Translation2d moduleTranslation;
            DriveModulePosition(int driveMotorID, int turnMotorID, InvertedValue driveInverted, double cancoderOffsetRotations, Translation2d moduleTranslation) {
                this.driveMotorID = driveMotorID;
                this.turnMotorID = turnMotorID;
                this.driveInverted = driveInverted;
                this.cancoderOffsetRotations = cancoderOffsetRotations;
                this.moduleTranslation = moduleTranslation;
            }

            public static final Translation2d[] moduleTranslations = Arrays.stream(values()).map((a) -> a.moduleTranslation).toArray(Translation2d[]::new);
        }

        /**Weight with battery and bumpers*/
        public static final double weightKg = Pounds.of(58.0).in(Kilograms);

        /**Distance between the front and back wheels*/
        public static final double trackWidthXMeters = Inches.of(25.5).in(Meters);
        /**Distance between the left and right wheels*/
        public static final double trackWidthYMeters = Inches.of(25.5).in(Meters);
        private static final double correctionVal = 314.0 / 320.55;
        public static final double wheelRadiusMeters = Inches.of(1.5).in(Meters) * correctionVal;

        public static final GearRatio driveWheelGearRatio = GearRatio.start(14).drive(22).driven(15).drive(45);
        public static final GearRatio turnWheelGearRatio = GearRatio.start(15).drive(32).driven(10).drive(60);
        public static final double driveWheelGearReduction = 1.0 / (1.0/4.0);
        public static final double turnWheelGearReduction = 1.0 / ((15.0/32.0)*(10.0/60.0));

        public static final double[] driveRealKps = {0.7, 0.4, 0.7, 0.7};
        public static final double[] driveRealKds = {3.5, 2.5, 3.7, 3.5};

        public static final double driveSnapKp = 1.5;
        public static final double driveSnapKi = 0;
        public static final double driveSnapKd = 0;

        public static final double maxDriveSpeedMetersPerSec = MetersPerSecond.of(6).in(MetersPerSecond);
        // TODO: Find the correct max acceleration (m)/(s^2)
        public static final double maxDriveAccelerationMetersPerSecPerSec = MetersPerSecondPerSecond.of(2).in(MetersPerSecondPerSecond);
        /**Tangential speed (m/s) = radial speed (rad/s) * radius (m)*/
        public static final double maxTurnRateRadiansPerSec = maxDriveSpeedMetersPerSec / Math.hypot(trackWidthXMeters/2, trackWidthYMeters/2);
        /**full speed in 0.25 sec*/
        public static final double joystickSlewRateLimit = 1.0 / 0.25;
        public static final double driveJoystickDeadbandPercent = 0.2;
        public static final double driveMaxJerk = 200.0;

        public static final double precisionLinearMultiplier = 0.2;
        public static final double precisionTurnMulitiplier = 0;//0.2;

        public static final double poseMoveTranslationkP = 1;
        public static final double poseMoveTranslationMaxVel = 3;
        public static final double poseMoveTranslationMaxAccel = 3;

        public static final double poseMoveRotationkP = 0.05;
        public static final double poseMoveRotationMaxVel = Math.PI;
        public static final double poseMoveRotationMaxAccel = Math.PI;

        public static final double headingKp = 0.2;
        public static final double headingKi = 0;
        public static final double headingKd = 0;
        public static final double headingTolerance = Degrees.of(1).in(Radians);

        public static final ModuleLimits moduleLimitsFree = new ModuleLimits(
            maxDriveSpeedMetersPerSec,
            maxDriveAccelerationMetersPerSecPerSec,
            maxTurnRateRadiansPerSec
        );

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveModulePosition.moduleTranslations);
    }

    public static final class PivotConstants {
        public static final double pivotMagnetOffset = 0.2099609375;//0.21337890625;
        public static final GearRatio motorToMechanismRatio = GearRatio
            .start(1).drive(5) // Planetary 1
            .driven(1).drive(5) // Planetary 2
            .driven(1).drive(4) // Planetary 3
            .driven(16).drive(36) // Chain
            ;
        public static final GearRatio encoderToMechanismRatio = GearRatio.start(1).drive(1);
        public static final GearRatio motorToEncoderRatio = motorToMechanismRatio.andThen(encoderToMechanismRatio.inverse());
    }

    public static final class ShooterConstants {
        public static final double exitVelocity = 5;

        public static final double[] distance = new double[] {
            FieldConstants.subwooferToSpeakerDist,
            FieldConstants.podiumToSpeakerDist
        };
        public static final double[] RPS = new double[] {
            90,
            90
        };
        public static final double[] angle = new double[] {
            Degrees.of(0).in(Radians),
            Degrees.of(22).in(Radians)
        };
    }

    public static final class VisionConstants {
        public static enum Camera {
            AprilTagVision(
                "AprilTagCam",
                new Transform3d(
                    new Translation3d(
                        Meters.of(0.349584),
                        Meters.of(0),
                        Meters.of(0.499868)
                    ),
                    new Rotation3d(
                        Units.degreesToRadians(-90),
                        -Units.degreesToRadians(30),
                        Units.degreesToRadians(0)
                    )
                )
            ),
            NoteVision(
                "NoteVisionCam",
                // Intake Transform
                // new Transform3d(
                //     new Translation3d(
                //         Centimeters.of(-73.5/2),
                //         Centimeters.of(20),
                //         Centimeters.of(24)
                //     ),
                //     new Rotation3d(
                //         0,
                //         0,
                //         Math.PI
                //     )
                // )
                // Backwards Pivot Transform
                // new Transform3d(
                //     new Translation3d(
                //         Centimeters.of(73.5/2-2.5),
                //         Centimeters.of(0),
                //         Centimeters.of(49)
                //     ),
                //     new Rotation3d(
                //         0,
                //         0,
                //         Math.PI
                //     )
                // )
                // CAD Transform
                new Transform3d(
                    new Translation3d(
                        Inches.of(-18.647965),
                        Inches.of(0),
                        Inches.of(-1.756603)
                    ),
                    new Rotation3d(
                        0,
                        Degrees.of(15).in(Radians),
                        Math.PI
                    )
                )
            ),
            ;
            public final String hardwareName;
            private final Transform3d intermediateToCamera;
            private Supplier<Transform3d> robotToIntermediate;
            Camera(String hardwareName, Transform3d finalToCamera) {
                this.hardwareName = hardwareName;
                this.intermediateToCamera = finalToCamera;
                this.robotToIntermediate = () -> new Transform3d();
            }
            private static Transform3d robotToCameraFromCalibTag(Transform3d robotToCalibTag, Transform3d cameraToCalibTag) {
                return robotToCalibTag.plus(cameraToCalibTag.inverse());
            }
            public Camera withRobotToIntermediate(Supplier<Transform3d> robotToFinal) {
                this.robotToIntermediate = robotToFinal;
                return this;
            }

            public Transform3d getRobotToCam() {
                return robotToIntermediate.get().plus(intermediateToCamera);
            }

            public ApriltagCamera toApriltagCamera() {
                return new ApriltagCamera(this.name(), new ApriltagCameraIO(){});
            }
            public ApriltagCamera toApriltagCamera(Function<Camera, ? extends ApriltagCameraIO> function) {
                return new ApriltagCamera(this.name(), function.apply(this));
            }

            public static void logCameraOverrides() {
                Logger.recordOutput("Camera Overrides", 
                    Arrays.stream(Camera.values())
                    .map(
                        (cam) -> 
                            new Transform3d(
                                new Pose3d(),
                                new Pose3d(RobotState.getInstance().getPose())
                            )
                            .plus(cam.getRobotToCam())
                    )
                    .toArray(Transform3d[]::new));
            }
        }

        // TODO: figure out vision stdDevs
        public static final double singleTagAmbiguityCutoff = 0.05;
        public static final double minimumStdDev = 0.5;
        public static final double stdDevEulerMultiplier = 0.3;
        public static final double stdDevDistanceMultiplier = 0.4;
    }

    public static final class AutoConstants {
        public static final double maxVel = 3;
        public static final double maxAccel = 3;

        public static final double maxVelFast = 4;
        public static final double maxAccelFast = 4.5;

        public static final double maxVelSlow = 0.75;
        public static final double maxAccelSlow = 1.5;

        public static final double autoTranslationXKp = 11;
        public static final double autoTranslationXKi = 0;
        public static final double autoTranslationXKd = 0;

        public static final double autoTranslationYKp = 8;
        public static final double autoTranslationYKi = 0;
        public static final double autoTranslationYKd = 0;

        public static final double autoTranslationSlowXKp = 8;
        public static final double autoTranslationSlowXKi = 0;
        public static final double autoTranslationSlowXKd = 0;

        public static final double autoTranslationSlowYKp = 6;
        public static final double autoTranslationSlowYKi = 0;
        public static final double autoTranslationSlowYKd = 0;

        public static final double autoRotationKp = 8;
        public static final double autoRotationKi = 0;
        public static final double autoRotationKd = 0;
    }

    public static final class FieldConstants {
        public static final double fieldLength = Units.inchesToMeters(648);
        public static final double fieldWidth =  Units.inchesToMeters(324);

        public static final Translation2d speakerCenter = new Translation2d(0.240581, 5.547755);

        public static final Pose2d ampFront = new Pose2d(new Translation2d(1.83, 7.61), Rotation2d.fromDegrees(90));
        public static final Pose2d speakerFront = new Pose2d(new Translation2d(1.45, 5.55), Rotation2d.fromDegrees(180));
        public static final Pose2d sourceFront = new Pose2d(new Translation2d(15.41, 1.04), Rotation2d.fromDegrees(120));
        public static final Pose2d podiumFront = new Pose2d(new Translation2d(2.54, 4.12), Rotation2d.fromDegrees(180));
        public static final Pose2d autoSpeakerFront = new Pose2d(new Translation2d(3.45, 5.55), Rotation2d.fromDegrees(180));
        public static final Pose2d autoSourceFront = new Pose2d(new Translation2d(13.41, 1.54), Rotation2d.fromDegrees(180));

        public static final double podiumToSpeakerDist = speakerCenter.getDistance(podiumFront.getTranslation());
        public static final double subwooferToSpeakerDist = speakerCenter.getDistance(speakerFront.getTranslation());
    }

    // Not the robot main function. This is called by Gradle when deploying to
    // make sure nobody deploys sim code.
    // Stolen from 6328 ;-)
    // See build.gradle

    /**
     * Checks that code is set to right mode when deploying
     */
    public static void main(String... args) {
        // if (getMode() != Mode.REAL) {
        //     System.err.println("Cannot deploy. Invalid mode: " + getMode());
        //     System.exit(1);
        // }
    }
}
