// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.swerve.ModuleLimits;
import frc.robot.util.swerve.SwerveSetpoint;
import frc.robot.util.swerve.SwerveSetpointGenerator;

public class Drive extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private Rotation2d prevGyroYaw = new Rotation2d();

    private final Module[] modules = new Module[DriveModulePosition.values().length]; // FL, FR, BL, BR

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.DriveModulePosition.moduleTranslations);
    private SwerveModuleState[] lastMeasuredStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private boolean isCharacterizing = false;
    private double characterizationVolts = 0.0;
    private final LoggedTunableNumber rotationCorrection = new LoggedTunableNumber("Drive/Rotation Correction", 0.25);

    private static final LoggedTunableNumber coastWaitTime =
        new LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
    private static final LoggedTunableNumber coastMetersPerSecThreshold =
        new LoggedTunableNumber("Drive/CoastMetersPerSecThreshold", 0.05);

    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private SwerveSetpoint currentSetpoint =
        new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        });
    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint prevSetpoint = currentSetpoint;

    private Timer lastMovementTimer = new Timer(); // used for brake mode

    private Twist2d fieldVelocity = new Twist2d();

    private final Timer currentSpikeTimer = new Timer();
    private static final LoggedTunableNumber currentSpikeThreshold = new LoggedTunableNumber("Drive/Current Spike Threshold", 0); 
    private static final LoggedTunableNumber currentSpikeTime = new LoggedTunableNumber("Drive/Current Spike Time", 0);
    private final ArrayList<Module> spikeModules = new ArrayList<>();

    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        System.out.println("[Init Drive] Instantiating Drive");
        this.gyroIO = gyroIO;
        System.out.println("[Init Drive] Gyro IO: " + this.gyroIO.getClass().getSimpleName());
        SmartDashboard.putData("Subsystems/Drive", this);
        ModuleIO[] moduleIOs = new ModuleIO[]{flModuleIO, frModuleIO, blModuleIO, brModuleIO};
        for(DriveModulePosition position : DriveModulePosition.values()) {
            System.out.println("[Init Drive] Instantiating Module " + position.name() + " with Module IO: " + moduleIOs[position.ordinal()].getClass().getSimpleName());
            modules[position.ordinal()] = new Module(moduleIOs[position.ordinal()], position.ordinal());
        }
        lastMovementTimer.start();
        for (var module : modules) {
            module.setBrakeMode(false);
        }

        setpointGenerator = new SwerveSetpointGenerator(DriveConstants.kinematics, DriveConstants.DriveModulePosition.moduleTranslations);

        // initialize pose estimator
        Pose2d initialPoseMeters = FieldConstants.speakerFront;
        RobotState.getInstance().initializePoseEstimator(kinematics, getGyroRotation(), getModulePositions(), initialPoseMeters);
        prevGyroYaw = getPose().getRotation();

        if (!AutoBuilder.isConfigured()) {
            AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::driveVelocity,
                autoConfigSup.get(),
                AllianceFlipUtil::shouldFlip,
                this
            );
        }
    }

    public void periodic() {
        // update IO inputs
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }

        ModuleLimits currentModuleLimits = DriveConstants.moduleLimitsFree;

        // drive train collision detection
        for (var module : modules) {
            if (Math.abs(module.getCurrentAmps()) >= currentSpikeThreshold.get()) {
                if (!spikeModules.contains(module)) {
                    spikeModules.add(module);
                }
            } else {
                spikeModules.remove(module);
            }
        }

        if (!spikeModules.isEmpty()) {
            currentSpikeTimer.start();
        } else {
            currentSpikeTimer.stop();
            currentSpikeTimer.reset();
        }

        // Log measured states
        SwerveModuleState[] measuredStates = new SwerveModuleState[DriveConstants.numDriveModules];
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            measuredStates[i] = modules[i].getState();
        }
        Logger.recordOutput("SwerveStates/Measured", measuredStates);
        lastMeasuredStates = measuredStates;

        // Update odometry
        Rotation2d gyroAngle;
        if (gyroInputs.connected) {
            gyroAngle = getGyroRotation();
        } else {
            // either the gyro is disconnected or we are in a simulation
            // accumulate a gyro estimate using module kinematics
            var wheelDeltas = getModulePositionDeltas(); // get change in module positions
            Twist2d twist = kinematics.toTwist2d(wheelDeltas); // dtheta will be the estimated change in chassis angle
            gyroAngle = prevGyroYaw.plus(Rotation2d.fromRadians(twist.dtheta));
        }
        RobotState.getInstance().addDriveMeasurement(gyroAngle, getModulePositions());

        // Logger.recordOutput("Odometry/Robot", getPose());

        // Update field velocity; use gyro when possible
        ChassisSpeeds robotRelativeVelocity = getChassisSpeeds();
        robotRelativeVelocity.omegaRadiansPerSecond =
            gyroInputs.connected
                ? gyroInputs.yawVelocityRadPerSec
                : robotRelativeVelocity.omegaRadiansPerSecond;
        fieldVelocity = GeomUtil.toTwist2d(robotRelativeVelocity);

        // Update brake mode
        if (Arrays.stream(modules)
            .anyMatch(module -> module.getVelocityMetersPerSec() > coastMetersPerSecThreshold.get())) {
            lastMovementTimer.reset();
        }

                // Run modules
        if (DriverStation.isDisabled()) {
            // Stop moving while disabled
            for (var module : modules) {
                module.stop();
            }

            // Clear setpoint logs
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});

        } else if (isCharacterizing) {
            // Run in characterization mode
            for (var module : modules) {
                module.runCharacterization(characterizationVolts);
            }

            // Clear setpoint logs
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        } else {
            currentSetpoint =
                setpointGenerator.generateSetpoint(currentModuleLimits, prevSetpoint, ChassisSpeeds.discretize(desiredSpeeds, rotationCorrection.get()), Constants.dtSeconds);

            SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
            for (int i = 0; i < modules.length; i++) {
                // Optimize setpoints
                optimizedSetpointStates[i] =
                    SwerveModuleState.optimize(currentSetpoint.moduleStates()[i], modules[i].getAngle());
                modules[i].runSetpoint(optimizedSetpointStates[i]);
            }

            prevSetpoint = currentSetpoint;

            Logger.recordOutput("SwerveStates/Setpoints", optimizedSetpointStates);
            Logger.recordOutput(
                "Drive/SwerveStates/Desired(b4 Poofs)",
                DriveConstants.kinematics.toSwerveModuleStates(desiredSpeeds));
            Logger.recordOutput("Drive/DesiredSpeeds", desiredSpeeds);
            Logger.recordOutput("Drive/SetpointSpeeds", currentSetpoint.chassisSpeeds());
        }

        // save values for next loop
        prevGyroYaw = gyroAngle;
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void driveVelocity(ChassisSpeeds speeds) {
        isCharacterizing = false;
        desiredSpeeds = speeds;
        // speeds will be applied next drive.periodic()
    }

    public void drivePercent(ChassisSpeeds speeds) {
        driveVelocity(new ChassisSpeeds(
                speeds.vxMetersPerSecond * DriveConstants.maxDriveSpeedMetersPerSec,
                speeds.vyMetersPerSecond * DriveConstants.maxDriveSpeedMetersPerSec,
                speeds.omegaRadiansPerSecond * DriveConstants.maxTurnRateRadiansPerSec));
    }

    /** Zeros the drive encoders. */
    public void zeroEncoders() {
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            modules[i].zeroEncoders();
        }
    }

    /** Stops the drive. */
    public void stop() {
        driveVelocity(new ChassisSpeeds());
    }

    public void setBrakeMode(Boolean enabled) {
        for (var module : modules) {
            module.setBrakeMode(enabled);
        }
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        stop();
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            prevSetpoint.moduleStates()[i] = new SwerveModuleState(
                prevSetpoint.moduleStates()[i].speedMetersPerSecond,
                DriveConstants.DriveModulePosition.moduleTranslations[i].getAngle()
            );
        }
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return DriveConstants.maxDriveSpeedMetersPerSec;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadiansPerSec() {
        return DriveConstants.maxTurnRateRadiansPerSec;
    }

    /**
     * Returns the measured X, Y, and theta field velocities in meters per sec. The
     * components of the twist are velocities and NOT changes in position.
     */
    public Twist2d getFieldVelocity() {
        return fieldVelocity;
    }

    /** Returns the current yaw (Z rotation). */
    public Rotation2d getGyroRotation() {
        return getYaw();
    }

    /** Returns the current yaw (Z rotation). */
    public Rotation2d getYaw() {
        return new Rotation2d(gyroInputs.yawPositionRad);
    }

    /** Returns the current pitch (Y rotation). */
    public Rotation2d getPitch() {
        return new Rotation2d(gyroInputs.pitchPositionRad);
    }

    /** Returns the current roll (X rotation). */
    public Rotation2d getRoll() {
        return new Rotation2d(gyroInputs.rollPositionRad);
    }

    /** Returns the current pitch velocity (Y rotation) in radians per second. */
    public double getPitchVelocity() {
        return gyroInputs.pitchVelocityRadPerSec;
    }

    /** Returns the current roll velocity (X rotation) in radians per second. */
    public double getRollVelocity() {
        return gyroInputs.rollVelocityRadPerSec;
    }

    /** Returns the current odometry pose. */
    public Pose2d getPose() {
        return RobotState.getInstance().getPose();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d newPose) {
        RobotState.getInstance().setPose(getGyroRotation(), getModulePositions(), newPose);
    }

    /** Adds vision data to the pose esimation. */
    // public void addVisionData(List<TimestampedVisionUpdate> visionData) {
    // poseEstimator.addVisionData(visionData);
    // }

    /** Returns an array of module positions. */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[DriveConstants.numDriveModules];
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            modulePositions[i] = modules[i].getPosition();
        }
        return modulePositions;
    }

    /** Returns an array of module positions. */
    public SwerveModulePosition[] getModulePositionDeltas() {
        SwerveModulePosition[] modulePositionDeltas = new SwerveModulePosition[DriveConstants.numDriveModules];
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            modulePositionDeltas[i] = modules[i].getPositionDelta();
        }
        return modulePositionDeltas;
    }

    /** Returns the average drive distance in radians */
    public double getAverageModuleDistance() {
        double avgDist = 0.0;
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            avgDist += Math.abs(modules[i].getPositionRadians());
        }
        return avgDist / DriveConstants.numDriveModules;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
    }

    /** Runs forwards at the commanded voltage. */
    public void runCharacterizationVolts(double volts) {
        isCharacterizing = true;
        characterizationVolts = volts;
    }

    /** Returns the average drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
            driveVelocityAverage += modules[i].getCharacterizationVelocity();
        }
        return driveVelocityAverage / DriveConstants.numDriveModules;
    }

    // field-oriented directions from driver's perspective
    public static enum CardinalDirection {
        FORWARD(Units.degreesToRadians(0)),
        BACKWARD(Units.degreesToRadians(180)),
        LEFT(Units.degreesToRadians(90)),
        RIGHT(Units.degreesToRadians(270));

        private final double angleRadians;

        private CardinalDirection(double angleRadians) {
            this.angleRadians = angleRadians;
        }

        public double getAngleRadians() {
            return angleRadians;
        }
    }

    // turn stick must exceed this threshold to change desired heading
    private static final double cardinalStickThreshold = 0.5;

    // use joystick to select cardinal direction
    public static Optional<CardinalDirection> getCardinalDirectionFromJoystick(DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        Optional<CardinalDirection> direction = Optional.empty();

        double xStick = xSupplier.getAsDouble();
        double yStick = ySupplier.getAsDouble();

        double xAbs = Math.abs(xStick);
        double yAbs = Math.abs(yStick);

        double maxStick = Math.max(xAbs, yAbs);
        if (maxStick > cardinalStickThreshold) {
            if (Math.abs(xStick) > Math.abs(yStick)) {
                direction = Optional.of(xStick > 0 ? CardinalDirection.LEFT : CardinalDirection.RIGHT);
            } else {
                direction = Optional.of(yStick > 0 ? CardinalDirection.FORWARD : CardinalDirection.BACKWARD);
            }
        }
        return direction;
    }

    // use joystick to select cardinal direction
    public static Optional<CardinalDirection> getCardinalDirectionFromButtons(
            BooleanSupplier forwardSupplier, BooleanSupplier backwardSupplier,
            BooleanSupplier leftSupplier, BooleanSupplier rightSupplier) {

        Optional<CardinalDirection> direction = Optional.empty();

        if (forwardSupplier.getAsBoolean()) {
            direction = Optional.of(CardinalDirection.FORWARD);
        } else if (backwardSupplier.getAsBoolean()) {
            direction = Optional.of(CardinalDirection.BACKWARD);
        } else if (leftSupplier.getAsBoolean()) {
            direction = Optional.of(CardinalDirection.LEFT);
        } else if (rightSupplier.getAsBoolean()) {
            direction = Optional.of(CardinalDirection.RIGHT);
        }
        return direction;
    }

    public boolean collisionDetected() {
        return currentSpikeTimer.hasElapsed(currentSpikeTime.get());
    }

    private static final LoggedTunableNumber tP = new LoggedTunableNumber("AutoDrive/tP", 1);
    private static final LoggedTunableNumber tI = new LoggedTunableNumber("AutoDrive/tI", 0);
    private static final LoggedTunableNumber tD = new LoggedTunableNumber("AutoDrive/tD", 0);
    private static final LoggedTunableNumber rP = new LoggedTunableNumber("AutoDrive/rP", 1.5);
    private static final LoggedTunableNumber rI = new LoggedTunableNumber("AutoDrive/rI", 0);
    private static final LoggedTunableNumber rD = new LoggedTunableNumber("AutoDrive/rD", 0);
    private static final double baseRadius = Arrays.stream(DriveConstants.DriveModulePosition.moduleTranslations).mapToDouble((t) -> t.getNorm()).max().orElse(0.5);
    public static final Supplier<HolonomicPathFollowerConfig> autoConfigSup = () -> {
        return new HolonomicPathFollowerConfig(
            new PIDConstants(
                tP.get(),
                tI.get(),
                tD.get()
            ),
            new PIDConstants(
                rP.get(),
                rI.get(),
                rD.get()
            ),
            DriveConstants.maxDriveSpeedMetersPerSec,
            baseRadius,
            new ReplanningConfig()
        );
    };

    public static final LoggedTunableNumber kAutoDriveMaxVelocity = new LoggedTunableNumber("Drive/kAutoDriveMaxVelocity", 1.0);
    public static final LoggedTunableNumber kMaxAcceleration = new LoggedTunableNumber("Drive/kMaxAcceleration", 1.0);
    public static final LoggedTunableNumber kMaxAngularVelocity = new LoggedTunableNumber("Drive/kMaxAngularVelocity", Units.degreesToRadians(540));
    public static final LoggedTunableNumber kMaxAngularAcceleration = new LoggedTunableNumber("Drive/kMaxAngularAcceleration", Units.degreesToRadians(720));
    private static final PathConstraints pathConstraints = new PathConstraints(
        kAutoDriveMaxVelocity.get(),
        kMaxAcceleration.get(),
        kMaxAngularVelocity.get(),
        kMaxAngularAcceleration.get()
    );

    private final Map<Pose2d, String> locationNames = new HashMap<>(Map.of(
        FieldConstants.ampFront, "amp",
        FieldConstants.speakerFront, "speaker",
        FieldConstants.podiumFront, "podium",
        FieldConstants.sourceFront, "source"
    ));

    public static String autoDrivePrefix = "AutoDrive";

    public Command driveToFlipped(Pose2d pos) {
        String name = locationNames.entrySet().stream()
            .filter(e -> e.getKey().equals(pos))
            .findFirst()
            .map(Map.Entry::getValue)
            .orElse(String.format("x %.3f, y %.3f, θ %.3f°", pos.getX(), pos.getY(), pos.getRotation().getDegrees()));

        return AutoBuilder.pathfindToPoseFlipped(pos, pathConstraints, 0, 0).withName(String.format("%s (%s)", autoDrivePrefix, name));
    }
}
