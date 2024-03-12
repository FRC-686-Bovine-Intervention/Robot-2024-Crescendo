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

import edu.wpi.first.math.MathUtil;
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
import frc.robot.util.GearRatio.Wheel;
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
        public static final int pivotLeftMotorID = 6;
        public static final int pivotRightMotorID = 7;
        public static final int pivotEncoderID = 6;

        // Kicker
        public static final int kickerLeftID = 8;
        public static final int kickerRightID = 9;

        // Shooter
        public static final int shooterLeftID = 8;
        public static final int shooterRightID = 9;
        public static final int shooterAmpID = 6;

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
        public static final int intakeSensorPort = 0;

        // Kicker
        public static final int kickerSensorPort = 1;
    }

    public static final class RobotConstants {
        public static final Rotation2d shooterForward = Rotation2d.fromDegrees(0);
        public static final Rotation2d intakeForward = Rotation2d.fromDegrees(180);

        /**Distance between the front and back wheels*/
        public static final double trackWidthXMeters = Inches.of(25.5).in(Meters);
        /**Distance between the left and right wheels*/
        public static final double trackWidthYMeters = Inches.of(25.5).in(Meters);
    }

    public static final class DriveConstants {
        public static enum DriveModulePosition {
            FRONT_LEFT(
                CANDevices.frontLeftDriveMotorID, CANDevices.frontLeftTurnMotorID,
                InvertedValue.CounterClockwise_Positive,
                0.75,
                new Translation2d(
                    +RobotConstants.trackWidthXMeters / 2.0,
                    +RobotConstants.trackWidthYMeters / 2.0
                )
            ),
            FRONT_RIGHT(
                CANDevices.frontRightDriveMotorID, CANDevices.frontRightTurnMotorID,
                InvertedValue.Clockwise_Positive,
                0.5,
                new Translation2d(
                    +RobotConstants.trackWidthXMeters / 2.0,
                    -RobotConstants.trackWidthYMeters / 2.0
                )
            ),
            BACK_LEFT(
                CANDevices.backLeftDriveMotorID, CANDevices.backLeftTurnMotorID,
                InvertedValue.CounterClockwise_Positive,
                0.5,
                new Translation2d(
                    -RobotConstants.trackWidthXMeters / 2.0,
                    +RobotConstants.trackWidthYMeters / 2.0
                )
            ),
            BACK_RIGHT(
                CANDevices.backRightDriveMotorID, CANDevices.backRightTurnMotorID,
                InvertedValue.Clockwise_Positive,
                0.75,
                new Translation2d(
                    -RobotConstants.trackWidthXMeters / 2.0,
                    -RobotConstants.trackWidthYMeters / 2.0
                )
            ),
            ;
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
        public static final int numDriveModules = DriveModulePosition.values().length;

        /**Weight with battery and bumpers*/
        public static final double weightKg = Pounds.of(58.0).in(Kilograms);
        
        public static final double driveBaseRadius = Arrays.stream(DriveModulePosition.moduleTranslations).mapToDouble((t) -> t.getNorm()).max().orElse(0.5);
        private static final double correctionVal = 314.0 / 320.55;
        public static final double wheelRadiusMeters = Inches.of(1.5).in(Meters) * correctionVal;

        public static final GearRatio driveWheelGearRatio = new GearRatio()
            .gear(14).gear(22).axle()
            .gear(15).gear(45).axle()
        ;
        public static final Wheel driveWheel = driveWheelGearRatio.wheelRadius(wheelRadiusMeters);
        public static final GearRatio turnWheelGearRatio = new GearRatio()
            .gear(15).gear(32).axle()
            .gear(10).gear(60).axle()
        ;
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
        public static final double maxTurnRateRadiansPerSec = maxDriveSpeedMetersPerSec / Math.hypot(RobotConstants.trackWidthXMeters/2, RobotConstants.trackWidthYMeters/2);
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
        public static final double pivotMagnetOffset = -0.332763671875;//-0.330322265625;//0.32958984375;
        public static final GearRatio motorToMechanismRatio = new GearRatio()
            .gear(+8).gear(+72).axle()
            .gear(+10).gear(+100).axle()
        ;
        public static final GearRatio encoderToMechanismRatio = new GearRatio()
            // .gear(+1).gear(+1).axle()
        ;
        public static final GearRatio motorToEncoderRatio = motorToMechanismRatio.concat(encoderToMechanismRatio.inverse());
    }

    public static final class ShooterConstants {
        public static final double exitVelocity = 5;

        public static final double wheelRadius = Inches.of(2).in(Meters);

        public static final Wheel motorToSurface = new GearRatio()
            .sprocket(+48).sprocket(+24)
            .wheelRadius(wheelRadius)
        ;
        public static final Wheel surfaceToMotor = motorToSurface.inverse();

        public static final double[] distance = new double[] {
            FieldConstants.subwooferToSpeakerDist,
            FieldConstants.podiumToSpeakerDist,
            Centimeters.of(565).minus(Inches.of(35)).in(Meters)
        };
        public static final double[] RPS = new double[] {
            30,
            65,
            65
        };
        public static final double[] angle = new double[] {
            Degrees.of(59.39).in(Radians),
            Degrees.of(37.8).in(Radians),
            Degrees.of(31.5).in(Radians)
        };
        public static double distLerp(double dist, double[] lerpTarget) {
            int lowerBound = 0;
            int upperBound = 0;
            for(int i = 0; i < distance.length; i++) {
                upperBound = i;
                if(dist < distance[i]) {
                    break;
                }
                lowerBound = i;
            }
            double t = MathUtil.inverseInterpolate(distance[lowerBound], distance[upperBound], dist);
            double target = MathUtil.interpolate(lerpTarget[lowerBound], lerpTarget[upperBound], t);
            return target;
        }
    }

    public static final class VisionConstants {
        public static enum Camera {
            LeftApriltag(
                "Left Apriltag Cam",
                new Transform3d(
                    new Translation3d(
                        Inches.of(+12.225),
                        Inches.of(+9.557),
                        Inches.of(+10.932)
                    ),
                    new Rotation3d(
                        Units.degreesToRadians(+(90.0-87.654)),
                        Units.degreesToRadians(-32.414),
                        Units.degreesToRadians(+9.707)
                    )
                )
            ),
            RightApriltag(
                "Right Apriltag Cam",
                new Transform3d(
                    new Translation3d(
                        Inches.of(+12.225),
                        Inches.of(-9.557),
                        Inches.of(+10.932)
                    ),
                    new Rotation3d(
                        Units.degreesToRadians(-(90.0-87.654)),
                        Units.degreesToRadians(-32.414),
                        Units.degreesToRadians(-9.707)
                    )
                )
            ),
            NoteVision(
                "Note Cam",
                new Transform3d(
                    new Translation3d(
                        Inches.of(-14.047),
                        Inches.of(+0),
                        Inches.of(+12.584)
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
                this.robotToIntermediate = Transform3d::new;
            }
            @SuppressWarnings("unused")
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

        public static final Translation2d speakerAimPoint = new Translation2d(0.240581, 5.547755);

        public static final Pose2d subwooferFront =     new Pose2d(new Translation2d(1.45, 5.55), Rotation2d.fromDegrees(+180));
        public static final Pose2d subwooferAmp =       new Pose2d(new Translation2d(0.71, 6.72), Rotation2d.fromDegrees(-120));
        public static final Pose2d subwooferSource =    new Pose2d(new Translation2d(0.71, 6.72), Rotation2d.fromDegrees(+120));
        
        public static final Pose2d amp =                new Pose2d(new Translation2d(1.83, 7.61), Rotation2d.fromDegrees(+90));
        public static final Pose2d podium =             new Pose2d(new Translation2d(2.76, 4.44), Rotation2d.fromDegrees(+157.47));

        public static final Pose2d pathfindSpeaker =    new Pose2d(new Translation2d(3.45, 5.55), Rotation2d.fromDegrees(+180));
        public static final Pose2d pathfindSource =     new Pose2d(new Translation2d(13.41, 1.54), Rotation2d.fromDegrees(+180));

        public static final double podiumToSpeakerDist =    speakerAimPoint.getDistance(podium.getTranslation());
        public static final double subwooferToSpeakerDist = speakerAimPoint.getDistance(subwooferFront.getTranslation());
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
