// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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

        // | Front Left: Red
        public static final int frontLeftDriveMotorID  = 11;
        public static final int frontLeftTurnMotorID   = 12;

        // | Front Right: Blue
        public static final int frontRightDriveMotorID  = 21;
        public static final int frontRightTurnMotorID   = 22;

        // | Back Left: Black
        public static final int backLeftDriveMotorID  = 41;
        public static final int backLeftTurnMotorID   = 42;

        // | Back Right: White
        public static final int backRightDriveMotorID  = 31;
        public static final int backRightTurnMotorID   = 32;

        // Arm
        public static final String armCanBusName = "rio";

        // | Elbow
        public static final int armMotorID          = 51;
        public static final int armEncoderID        = 52;

        // | Manipulator
        public static final int manipMotorID        = 53;

        // Bunny Intake
        public static final int bunnyIntakeMotorID  = 61;


        public static final int pigeonCanID = 0;
        public static final int candleCanID = 0;

        public static final double minCanUpdateRate = 4.0;
    }

    public static final class DIOPorts {

        public static final int brakeSwitchPort = 1;
        public static final int ledSwitchPort = 2;
    }

    public static final class DriveConstants {
        public static int numDriveModules = 4;
        public static enum DriveModulePosition {
            FRONT_LEFT  (CANDevices.frontLeftDriveMotorID, CANDevices.frontLeftTurnMotorID, InvertedValue.CounterClockwise_Positive,
            0.75),
            FRONT_RIGHT (CANDevices.frontRightDriveMotorID, CANDevices.frontRightTurnMotorID, InvertedValue.Clockwise_Positive,
            0.5),
            BACK_LEFT   (CANDevices.backLeftDriveMotorID, CANDevices.backLeftTurnMotorID, InvertedValue.CounterClockwise_Positive,
            0.5),
            BACK_RIGHT  (CANDevices.backRightDriveMotorID, CANDevices.backRightTurnMotorID, InvertedValue.Clockwise_Positive,
            0.75);
            public final int driveMotorID;
            public final int turnMotorID;
            // motor direction to drive 'forward' (cancoders at angles given in cancoderOffsetRotations)
            public final InvertedValue driveInverted;
            // absolute position of cancoder when drive wheel is facing 'forward'
            public final double cancoderOffsetRotations;
            DriveModulePosition(int driveMotorID, int turnMotorID, InvertedValue driveInverted, double cancoderOffsetRotations) {
                this.driveMotorID = driveMotorID;
                this.turnMotorID = turnMotorID;
                this.driveInverted = driveInverted;
                this.cancoderOffsetRotations = cancoderOffsetRotations;
            }
        }

        // weight with battery and bumpers
        public static final double weightKg = Units.lbsToKilograms(58.0);

        public static final double trackWidthXMeters = Units.inchesToMeters(25.5); // distance between the front and back wheels
        public static final double trackWidthYMeters = Units.inchesToMeters(25.5); // distance between the left and right wheels
        public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);

        public static final double driveWheelGearReduction = 1.0 / ((14.0/22.0)*(15.0/45.0));
        public static final double turnWheelGearReduction = 1.0 / ((15.0/32.0)*(10.0/60.0));

        public static final double[] driveRealKps = {0.7, 0.4, 0.7, 0.7};
        public static final double[] driveRealKds = {3.5, 2.5, 3.7, 3.5};

        public static final double driveSnapKp = 1.5;
        public static final double driveSnapKi = 0;
        public static final double driveSnapKd = 0;


        public static final double maxDriveSpeedMetersPerSec = 4;
        // tangential speed (m/s) = radial speed (rad/s) * radius (m)
        public static final double maxTurnRateRadiansPerSec = maxDriveSpeedMetersPerSec / Math.hypot(trackWidthXMeters/2, trackWidthYMeters/2);

        public static final double joystickSlewRateLimit = 1.0 / 0.25;     // full speed in 0.25 sec
        public static final double driveJoystickDeadbandPercent = 0.2;
        public static final double driveMaxJerk = 200.0;

        public static final double precisionLinearMultiplier = 0.2;
        public static final double precisionTurnMulitiplier = 0.2;

        public static final double poseMoveTranslationkP = 1;
        public static final double poseMoveTranslationMaxVel = 3;
        public static final double poseMoveTranslationMaxAccel = 3;

        public static final double poseMoveRotationkP = 0.05;
        public static final double poseMoveRotationMaxVel = Math.PI;
        public static final double poseMoveRotationMaxAccel = Math.PI;

    }

    public static final class ArmConstants {
        public static final double elbowMotorToJointGearRatio = 1;
    }

    public static final class VisionConstants {
        public static enum Camera {
            Front(
                "Front",
                new Transform3d(new Translation3d(95.75 / 100, 0, 13.25 / 100), new Rotation3d(0, 0, Units.degreesToRadians(180))),
                new Transform3d(new Translation3d(0.7695, 0.1391, -0.1402), new Rotation3d(new Quaternion(0.022, 0.1079, 0.0035, -0.9939)))
            ),
            Back(
                "Back",
                new Transform3d(new Translation3d(-95.75 / 100, 0, 13.25 / 100), new Rotation3d(0, 0, Units.degreesToRadians(0))),
                new Transform3d(new Translation3d(1.171, 0.0415, -0.2687), new Rotation3d(new Quaternion(0.0197, 0.1799, 0, -0.9835)))
            ),
            Limelight(
                "limelight",
                new Transform3d(new Translation3d(-95.75 / 100, 0, 13.25 / 100), new Rotation3d(0, 0, Units.degreesToRadians(0))),
                new Transform3d(new Translation3d(1.171, 0.0415, -0.2687), new Rotation3d(new Quaternion(0.0197, 0.1799, 0, -0.9835)))
            ),
            ;
            public final String hardwareName;
            public final Transform3d robotToCamera;
            Camera(String hardwareName, Transform3d robotToCalibTag, Transform3d cameraToCalibTag) {
                this.hardwareName = hardwareName;
                this.robotToCamera = robotToCalibTag.plus(cameraToCalibTag.inverse());
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

        public static final double autoBalanceKp = 0.4;
        public static final double autoBalanceKi = 0.05;
        public static final double autoBalanceKd = 0.0;

        public static final double initialBalanceSpeed = 1;

    }


   // public static final class LEDConstants {

    //     // Ports
    //     public static final int ledPort = 0;

    //     // LED Data
    //     public static final int armLedCount = 123;
    //     public static final int baseLedCount = 128;

    //     // Rainbow
    //     public static final boolean dynamicRainbow = true;
    //     public static final int dynamicRainbowSpeed = 1;

    //     // Pre-Match Climb Pattern
    //     public static final int climbSpeed = 2;
    //     public static final int climbMaxDelay = 40;
    //     public static final int climbMinDelay = 20;
    //     public static final int climbMaxLength = 10;
    //     public static final int climbMinLength = 5;

    //     // Other
    //     public static final Color activeSideFlashColor = new Color(0, 0, 0);
    //     public static final Color intakeFlashColor = new Color(255, 255, 255);
    //     public static final Color whistleFlashColor = new Color(255, 179, 0);

    // }

    public static final class OverrideConstants {
        public static final int armBrakeModeButton = 3;
        public static final int driveBrakeModeButton = 9;
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
