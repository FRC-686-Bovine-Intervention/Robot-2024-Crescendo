package frc.robot.commands;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Intake.IntakeCommand;
import frc.robot.subsystems.vision.note.NoteVision.TrackedNote;
import frc.robot.util.LazyOptional;
import frc.robot.util.VirtualSubsystem;

public class AutoIntake extends VirtualSubsystem {
    private final Supplier<List<TrackedNote>> trackedNotes;

    private Optional<TrackedNote> optTarget = Optional.empty();

    private static final double confidenceThreshold = 1;

    public AutoIntake(Supplier<List<TrackedNote>> trackedNotes, Consumer<TrackedNote> forget) {
        this.trackedNotes = trackedNotes;

        CommandScheduler.getInstance().onCommandFinish((comm) -> {if (comm.getName() == IntakeCommand.INTAKE.name()) {
            optTarget.ifPresent((target) -> forget.accept(target));
            optTarget = Optional.empty();
        }});
    }

    @Override
    public void periodic() {
        if(optTarget.isEmpty()) {
            optTarget = trackedNotes.get().stream().filter((target) -> target.confidence >= confidenceThreshold).sorted((a,b) -> (int)Math.signum(a.confidence - b.confidence)).findFirst();
        }
    }

    public Supplier<ChassisSpeeds> getTranslationalSpeeds(Supplier<ChassisSpeeds> joystickFieldRelative) {
        return () -> optTarget.map((target) -> {
            var robotTrans = RobotState.getInstance().getPose().getTranslation();
            var targetRelRobot = target.fieldPos.minus(robotTrans);
            var targetRelRobotNormalized = targetRelRobot.div(targetRelRobot.getNorm());
            var joystickSpeed = joystickFieldRelative.get();
            var joy = new Translation2d(joystickSpeed.vxMetersPerSecond, joystickSpeed.vyMetersPerSecond);
            var boy = joy.div(DriveConstants.maxDriveSpeedMetersPerSec);
            var magnitude = dotProduct(targetRelRobotNormalized, boy);
            var finalTrans = targetRelRobotNormalized.times(DriveConstants.maxDriveSpeedMetersPerSec * magnitude);
            return new ChassisSpeeds(finalTrans.getX(), finalTrans.getY(), 0);
        }).orElseGet(joystickFieldRelative);
    }

    public LazyOptional<Translation2d> targetLocation() {
        return () -> optTarget.map((target) -> target.fieldPos);
    }

    private static double dotProduct(Translation2d a, Translation2d b) {
        return a.getX()*b.getX() + a.getY()*b.getY();
    }
}
