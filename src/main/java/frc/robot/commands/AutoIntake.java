package frc.robot.commands;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Intake.IntakeCommand;
import frc.robot.subsystems.vision.note.NoteVision.TrackedNote;
import frc.robot.util.LazyOptional;
import frc.robot.util.MathExtraUtil;
import frc.robot.util.VirtualSubsystem;

public class AutoIntake extends VirtualSubsystem {
    private static final double acquireConfidenceThreshold = 1;
    private static final double detargetConfidenceThreshold = 0.5;
    private final Supplier<List<TrackedNote>> trackedNotes;

    private Optional<TrackedNote> optTarget = Optional.empty();
    private boolean locked = false;

    public AutoIntake(Supplier<List<TrackedNote>> trackedNotes, Consumer<TrackedNote> forget) {
        this.trackedNotes = trackedNotes;

        CommandScheduler.getInstance().onCommandFinish((comm) -> {if (comm.getName() == IntakeCommand.INTAKE.name()) {
            optTarget.ifPresent((target) -> forget.accept(target));
            optTarget = Optional.empty();
        }});
    }

    @Override
    public void periodic() {
        if(optTarget.isPresent() && optTarget.get().confidence < detargetConfidenceThreshold) {
            optTarget = Optional.empty();
        }
        if(optTarget.isEmpty() || !locked) {
            optTarget = trackedNotes.get().stream().filter((target) -> target.confidence >= acquireConfidenceThreshold || DriverStation.isAutonomousEnabled()).sorted((a,b) -> (int)Math.signum(a.confidence - b.confidence)).findFirst();
        }
        Logger.recordOutput("DEBUG/NoteVision/Locked", locked);
        locked = false;
        Logger.recordOutput("DEBUG/NoteVision/Has target", optTarget.isPresent());
        Logger.recordOutput("DEBUG/NoteVision/Target", optTarget.map((target) -> target.fieldPos).orElse(null));
    }

    public DoubleSupplier applyDotProduct(Supplier<ChassisSpeeds> joystickFieldRelative) {
        return () -> optTarget.map((target) -> {
            var robotTrans = RobotState.getInstance().getPose().getTranslation();
            var targetRelRobot = target.fieldPos.minus(robotTrans);
            var targetRelRobotNormalized = targetRelRobot.div(targetRelRobot.getNorm());
            var joystickSpeed = joystickFieldRelative.get();
            var joy = new Translation2d(joystickSpeed.vxMetersPerSecond, joystickSpeed.vyMetersPerSecond);
            var throttle = MathExtraUtil.dotProduct(targetRelRobotNormalized, joy);
            return throttle;
        }).orElse(0.0);
    }

    public LazyOptional<ChassisSpeeds> getTransSpeed(DoubleSupplier throttleSupplier) {
        return () -> optTarget.map((target) -> {
            var robotTrans = RobotState.getInstance().getPose().getTranslation();
            var targetRelRobot = target.fieldPos.minus(robotTrans);
            var targetRelRobotNormalized = targetRelRobot.div(targetRelRobot.getNorm());
            var finalTrans = targetRelRobotNormalized.times(throttleSupplier.getAsDouble());
            return new ChassisSpeeds(finalTrans.getX(), finalTrans.getY(), 0);
        });
    }

    public LazyOptional<Translation2d> targetLocation() {
        return () -> optTarget.map((target) -> {
            locked = true;
            return target.fieldPos;
        });
    }
}
