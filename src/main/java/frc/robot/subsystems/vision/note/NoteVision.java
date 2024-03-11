package frc.robot.subsystems.vision.note;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeCommand;
import frc.robot.util.LazyOptional;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MathExtraUtil;
import frc.robot.util.VirtualSubsystem;

public class NoteVision extends VirtualSubsystem {
    private final NoteVisionIO noteVisionIO;
    private final NoteVisionIOInputsAutoLogged inputs = new NoteVisionIOInputsAutoLogged();

    private final ArrayList<TrackedNote> noteMemories = new ArrayList<>();

    private static final LoggedTunableNumber updateDistanceThreshold = new LoggedTunableNumber("Vision/Note/updateDistanceThreshold", 5);
    private static final LoggedTunableNumber posUpdatingFilteringFactor = new LoggedTunableNumber("Vision/Note/posUpdatingFilteringFactor", 0.8);
    private static final LoggedTunableNumber confUpdatingFilteringFactor = new LoggedTunableNumber("Vision/Note/posUpdatingFilteringFactor", 0.5);
    public static final LoggedTunableNumber confidencePerAreaPercent = new LoggedTunableNumber("Vision/Note/confidencePerAreaPercent", 1);
    private static final LoggedTunableNumber confidenceDecayPerSecond = new LoggedTunableNumber("Vision/Note/confidenceDecayPerSecond", 1);

    private static final double acquireConfidenceThreshold = 1;
    private static final double detargetConfidenceThreshold = 0.5;

    private Optional<TrackedNote> optIntakeTarget = Optional.empty();
    private boolean intakeTargetLocked = false;

    public NoteVision(NoteVisionIO noteVisionIO) {
        System.out.println("[Init NoteVision] Instantiating NoteVision");
        this.noteVisionIO = noteVisionIO;
        System.out.println("[Init NoteVision] NoteVision IO: " + this.noteVisionIO.getClass().getSimpleName());

        CommandScheduler.getInstance().onCommandFinish((comm) -> {if (comm.getName() == IntakeCommand.INTAKE.name()) {
            optIntakeTarget.ifPresent((target) -> noteMemories.remove(target));
            optIntakeTarget = Optional.empty();
        }});
    }

    @Override
    public void periodic() {
        noteVisionIO.updateInputs(inputs);
        Logger.processInputs("NoteVision", inputs);
        var frameTargets = Arrays.asList(inputs.trackedNotes);
        var connections = new ArrayList<PhotonMemoryConnection>();
        noteMemories.forEach(
            (memory) -> frameTargets.forEach(
                (target) -> {
                    if(memory.fieldPos.getDistance(target.fieldPos) < updateDistanceThreshold.get()) {
                        connections.add(new PhotonMemoryConnection(memory, target));
                    }
                }
            )
        );
        connections.sort((a, b) -> (int) Math.signum(a.getDistance() - b.getDistance()));
        var unusedMemories = new ArrayList<>(noteMemories);
        var unusedTargets = new ArrayList<>(frameTargets);
        while(!connections.isEmpty()) {
            var confirmedConnection = connections.get(0);
            confirmedConnection.memory.updatePosWithFiltering(confirmedConnection.photonFrameTarget);
            confirmedConnection.memory.updateConfidence();
            unusedMemories.remove(confirmedConnection.memory);
            unusedTargets.remove(confirmedConnection.photonFrameTarget);
            connections.removeIf((connection) -> 
                connection.memory == confirmedConnection.memory
                ||
                connection.photonFrameTarget == confirmedConnection.photonFrameTarget
            );
        }
        unusedMemories.forEach((memory) -> {
            if(RobotState.getInstance().getPose().getTranslation().getDistance(memory.fieldPos) > 1) {
                memory.decayConfidence(memory.isWithinView() ? 10 : 1);
            }
        });
        unusedTargets.forEach((target) -> noteMemories.add(target));
        noteMemories.removeIf((memory) -> memory.confidence <= 0);
        noteMemories.removeIf((memory) -> Double.isNaN(memory.fieldPos.getX()));

        if(optIntakeTarget.isPresent() && optIntakeTarget.get().confidence < detargetConfidenceThreshold) {
            optIntakeTarget = Optional.empty();
        }
        if(optIntakeTarget.isEmpty() || !intakeTargetLocked) {
            optIntakeTarget = noteMemories.stream().filter((target) -> target.confidence >= acquireConfidenceThreshold).sorted((a,b) -> (int)Math.signum(a.confidence - b.confidence)).findFirst();
        }
        intakeTargetLocked = false;

        // Logger.recordOutput("Vision/Note/Photon Frame Targets", frameTargets.stream().map(NoteVision::targetToPose).toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Note/Note Memories", noteMemories.stream().map(NoteVision::targetToPose).toArray(Pose3d[]::new));
        Logger.recordOutput("Vision/Note/Note Confidence", noteMemories.stream().mapToDouble((note) -> note.confidence).toArray());
    }

    public DoubleSupplier applyDotProduct(Supplier<ChassisSpeeds> joystickFieldRelative) {
        return () -> optIntakeTarget.map((target) -> {
            var robotTrans = RobotState.getInstance().getPose().getTranslation();
            var targetRelRobot = target.fieldPos.minus(robotTrans);
            var targetRelRobotNormalized = targetRelRobot.div(targetRelRobot.getNorm());
            var joystickSpeed = joystickFieldRelative.get();
            var joy = new Translation2d(joystickSpeed.vxMetersPerSecond, joystickSpeed.vyMetersPerSecond);
            var throttle = MathExtraUtil.dotProduct(targetRelRobotNormalized, joy);
            return throttle;
        }).orElse(0.0);
    }

    public LazyOptional<ChassisSpeeds> getAutoIntakeTransSpeed(DoubleSupplier throttleSupplier) {
        return () -> optIntakeTarget.map((target) -> {
            var robotTrans = RobotState.getInstance().getPose().getTranslation();
            var targetRelRobot = target.fieldPos.minus(robotTrans);
            var targetRelRobotNormalized = targetRelRobot.div(targetRelRobot.getNorm());
            var finalTrans = targetRelRobotNormalized.times(throttleSupplier.getAsDouble());
            return new ChassisSpeeds(finalTrans.getX(), finalTrans.getY(), 0);
        });
    }

    public LazyOptional<Translation2d> autoIntakeTargetLocation() {
        return () -> optIntakeTarget.map((target) -> {
            intakeTargetLocked = true;
            return target.fieldPos;
        });
    }

    public boolean hasTarget() {
        return optIntakeTarget.isPresent();
    }

    public void clearMemory() {
        noteMemories.clear();
    }

    public Command autoIntake(DoubleSupplier throttle, Drive drive, Intake intake) {
        return 
            drive.translationSubsystem.fieldRelative(getAutoIntakeTransSpeed(throttle).orElseGet(ChassisSpeeds::new))
            .alongWith(
                drive.rotationalSubsystem.pidControlledHeading(Drive.Rotational.pointTo(autoIntakeTargetLocation(), () -> RobotConstants.intakeForward))
            )
            .onlyWhile(() -> !intake.hasNote())
            .withName("Auto Intake")
        ;
    }

    private static Pose3d targetToPose(TrackedNote note) {
        return new Pose3d(new Translation3d(note.fieldPos.getX(), note.fieldPos.getY(), Units.inchesToMeters(1)), new Rotation3d());
    }

    private static record PhotonMemoryConnection(TrackedNote memory, TrackedNote photonFrameTarget) {
        public double getDistance() {
            return memory.fieldPos.getDistance(photonFrameTarget.fieldPos);
        }
    }

    public static class TrackedNote implements StructSerializable {
        public Translation2d fieldPos;
        public double confidence;

        public TrackedNote(Translation2d fieldPos, double confidence) {
            this.fieldPos = fieldPos;
            this.confidence = confidence * confUpdatingFilteringFactor.get();
        }

        public void updateConfidence() {
            confidence += confidence * MathUtil.clamp(1 - confUpdatingFilteringFactor.get(), 0, 1); 
        }

        public void updatePosWithFiltering(TrackedNote newNote) {
            this.fieldPos = fieldPos.interpolate(newNote.fieldPos, posUpdatingFilteringFactor.get());
            this.confidence = newNote.confidence;
        }

        public boolean isWithinView() {
            return false;
                // Math.abs() < FOVYawThreshold.get() &&
                // fieldPos.getDistance(new Pose3d(RobotState.getInstance().getPose()).toPose2d().getTranslation()) < FOVDistanceThreshold.get();
        }

        public void decayConfidence(double rate) {
            this.confidence -= confidenceDecayPerSecond.get() * rate * Constants.dtSeconds;
        }

        public static final TrackedNoteStruct struct = new TrackedNoteStruct();
        public static class TrackedNoteStruct implements Struct<TrackedNote> {
            @Override
            public Class<TrackedNote> getTypeClass() {
                return TrackedNote.class;
            }

            @Override
            public String getTypeString() {
                return "struct:TrackedNote";
            }

            @Override
            public int getSize() {
                return kSizeDouble + Translation2d.struct.getSize();
            }

            @Override
            public String getSchema() {
                return "Translation2d fieldPos;double confidence";
            }

            @Override
            public Struct<?>[] getNested() {
                return new Struct<?>[] {Translation2d.struct};
            }

            @Override
            public TrackedNote unpack(ByteBuffer bb) {
                var fieldPos = Translation2d.struct.unpack(bb);
                var confidence = bb.getDouble();
                return new TrackedNote(fieldPos, confidence);
            }

            @Override
            public void pack(ByteBuffer bb, TrackedNote value) {
                Translation2d.struct.pack(bb, value.fieldPos);
                bb.putDouble(value.confidence);
            }
        }
    }
}
