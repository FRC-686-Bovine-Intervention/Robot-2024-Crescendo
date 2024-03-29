package frc.robot.auto;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Alert;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.MathExtraUtil;
import frc.robot.util.Alert.AlertType;

public class AutoCommons {
    public static enum StartPosition {
        SubwooferFront(FieldConstants.subwooferFront),
        SubwooferAmp(FieldConstants.subwooferAmp),
        SubwooferSource(FieldConstants.subwooferSource),
        Amp(new Pose2d(
            new Translation2d(
                1.40,
                6.80
            ),
            Rotation2d.fromDegrees(180)
        )),
        Podium(new Pose2d(
            new Translation2d(
                1.40,
                4.10
            ),
            Rotation2d.fromDegrees(180)
        )),
        Source(new Pose2d(
            new Translation2d(
                1.40,
                3.30
            ),
            Rotation2d.fromDegrees(180)
        ))
        ;
        public final Pose2d startPose;
        StartPosition(Pose2d startPose) {
            this.startPose = startPose;
        }
    }

    public static Command setOdometryFlipped(Pose2d pose, Drive drive) {
        return Commands.runOnce(() -> RobotState.getInstance().setPose(drive.getGyroRotation(), drive.getModulePositions(), AllianceFlipUtil.apply(pose)));
    }

    public static Command followPathFlipped(PathPlannerPath path, Drive drive) {
        return new FollowPathHolonomic(path, drive::getPose, drive::getChassisSpeeds, drive::driveVelocity, Drive.autoConfigSup.get(), AllianceFlipUtil::shouldFlip, drive.translationSubsystem, drive.rotationalSubsystem);
    }
    public static Command followPathFlipped(PathPlannerPath path, Drive.Translational drive) {
        return new FollowPathHolonomic(path, drive.drive::getPose, drive.drive::getChassisSpeeds, drive::driveVelocity, Drive.autoConfigSup.get(), AllianceFlipUtil::shouldFlip, drive);
    }

    public static Command shootWhenReady(Translation2d pos, Drive drive, Shooter shooter, Pivot pivot, Kicker kicker) {
        var FORR = getFORR(pos);
        var dist = FORR.getNorm();
        var shootPos = new Pose2d(pos, new Rotation2d(FORR.getX(), FORR.getY()));
        BooleanSupplier condition = () -> 
            // kicker.hasNote() && 
            shooter.readyToShoot() && 
            pivot.isAtAngle(ShooterConstants.distLerp(dist, ShooterConstants.angle)) && 
            MathExtraUtil.isNear(shootPos, drive.getPose(), 0.75, Units.degreesToRadians(3)) && 
            MathExtraUtil.isNear(new ChassisSpeeds(), drive.getChassisSpeeds(), 0.5, 0.2)
        ;
        return kicker.kick().asProxy().onlyWhile(condition).onlyIf(condition).repeatedly().until(kicker::sensorFallingEdge);
        // return Commands.waitUntil(() -> 
        //     // kicker.hasNote() && 
        //     shooter.readyToShoot() && 
        //     pivot.isAtAngle(ShooterConstants.distLerp(dist, ShooterConstants.angle)) && 
        //     MathExtraUtil.isNear(shootPos, drive.getPose(), 0.75, Units.degreesToRadians(3)) && 
        //     MathExtraUtil.isNear(new ChassisSpeeds(), drive.getChassisSpeeds(), 0.5, 0.2)
        // )
        // .andThen(kicker.kick().asProxy());
    }

    private static Translation2d getFORR(Translation2d pos) {
        return AllianceFlipUtil.apply(FieldConstants.speakerAimPoint).minus(pos);
    }
    public static Command autoAim(Translation2d pos, Drive.Rotational rotation) {
        return rotation.pidControlledHeading(() -> Optional.of(getFORR(pos)).map((t) -> new Rotation2d(t.getX(), t.getY())));
    }
    public static Command autoAim(Translation2d pos, Shooter shooter, Kicker kicker) {
        return shooter.shoot(() -> getFORR(pos), kicker::sensorFallingEdge).asProxy();
    }
    public static Command autoAim(Translation2d pos, Pivot pivot) {
        return pivot.autoAim(() -> getFORR(pos)).asProxy();
    }
    public static Command autoAim(Translation2d pos, Shooter shooter, Kicker kicker, Pivot pivot) {
        return autoAim(pos, shooter, kicker).alongWith(autoAim(pos, pivot));
    }
    public static Command autoAim(Translation2d pos, Shooter shooter, Kicker kicker, Pivot pivot, Drive.Rotational rotation) {
        return autoAim(pos, shooter, kicker, pivot).alongWith(autoAim(pos, rotation));
    }

    public static class AutoPaths {
        // public static final String toCenterLine = "%s Start";
        // public static final String fromCenterLine = "%s Back";

        public static final String startToSpike = "%s Start to Spike";

        private static final Map<String, PathPlannerPath> loadedPaths = new HashMap<>();
        private static boolean preloading;
        public static void preload() {
            preloading = true;
            // loadPath(String.format(AutoPaths.startToSpike, "Amp"));
            loadPath("MASW Amp Start to Spike");
            loadPath("MASW Amp Spike to Center Spike");
            loadPath("MASW Center Spike to Podium Spike");
            loadPath("MASW Podium Spike to Center");
            loadPath("R6N Amp Start to Spike");
            loadPath("R6N Amp Spike to Center");
            loadPath("R6N Center to Amp Wing");
            loadPath("R6N Amp Wing to Center");
            preloading = false;
            System.out.println("[Init AutoPaths] Loaded paths");
        }

        public static PathPlannerPath loadPath(String name) {
            if(loadedPaths.containsKey(name)) {
                return loadedPaths.get(name);
            } else {
                if(!preloading) new Alert("[AutoPaths] Loading \"" + name + "\" which wasn't preloaded. Please add path to AutoPaths.preload()", AlertType.WARNING).set(true);
                var path = PathPlannerPath.fromPathFile(name);
                loadedPaths.put(name, path);
                return path;
            }
        }
    }
}
