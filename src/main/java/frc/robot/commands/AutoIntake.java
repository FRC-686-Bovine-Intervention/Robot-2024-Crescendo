package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.note.NoteVision;
import frc.robot.subsystems.vision.note.NoteVision.TrackedNote;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AllianceFlipUtil.FieldFlipType;
import frc.robot.util.controllers.Joystick;

public class AutoIntake extends Command {
    private final Drive drive;
    private final Intake intake;
    private final NoteVision noteVision;
    private final Joystick translationalJoystick;

    private final PIDController thetaPID = new PIDController(2, 0, 0);

    private Optional<TrackedNote> optTarget = Optional.empty();

    public AutoIntake(Joystick translationalJoystick, Drive drive, Intake intake, NoteVision noteVision) {
        addRequirements(drive);
        this.drive = drive;
        this.intake = intake;
        this.noteVision = noteVision;
        this.translationalJoystick = translationalJoystick;
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        optTarget = Optional.empty();
    }

    @Override
    public void execute() {
        if(optTarget.isEmpty()) {
            optTarget = noteVision.getTrackedNotes().stream().sorted((a,b) -> (int)Math.signum(a.confidence - b.confidence)).findFirst();
            if(optTarget.isEmpty()) return;
        }
        var target = optTarget.get();
        var robotPose = drive.getPose();
        var robotTrans = robotPose.getTranslation();
        var robotRot = robotPose.getRotation();
        var targetRelRobot = target.fieldPos.minus(robotTrans);
        var targetRelRobotNormalized = targetRelRobot.div(targetRelRobot.getNorm());
        var magnitude = dotProduct(targetRelRobot, AllianceFlipUtil.apply(new Translation2d(translationalJoystick.y().getAsDouble(), -translationalJoystick.x().getAsDouble()), FieldFlipType.CenterPointFlip));
        var idkanymore = targetRelRobotNormalized.times(drive.getMaxLinearSpeedMetersPerSec() * magnitude);
        drive.driveVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                idkanymore.getX(),
                idkanymore.getY(),
                thetaPID.calculate(robotRot.getRadians(), Math.atan2(-targetRelRobot.getY(), -targetRelRobot.getX()))
            ),
            drive.getRotation()
        ));
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return optTarget.isEmpty() || intake.hasNote();
    }

    private static double dotProduct(Translation2d a, Translation2d b) {
        return a.getX()*b.getX() + a.getY()*b.getY();
    }
}
