// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.MathExtraUtil;

public class ScoreNote extends AutoRoutine {
    private final static int MAX_QUESTION_COUNT = 2 + 3 * 2;

    private enum StartPosition {
        Amp,
        Speaker,
        Source
    }

    private enum Count {
        zero,
        one,
        two,
        three,
    }

    private enum Note {
        note_1,
        note_2,
        note_3,
        note_4,
        note_5,
        note_6,
        note_7,
        note_8
    }

    private enum ScorePosition {
        Speaker,
        Podium,
        Existing
    }

    
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", StartPosition::values);

    private static final AutoQuestion<Count> numberNotes = new AutoQuestion<>("Number of Notes", Count::values);
    private static final List<AutoQuestion<Note>> notes = new ArrayList<>();
    private static final List<AutoQuestion<ScorePosition>> scorePositions = new ArrayList<>();

    private static final String startToNoteTemplate = "%s Start Note %s";
    private static final String startDriveTemplate = "%s Start Drive";
    private static final String noteToNoteTemplate = "Note %s Note %s";
    private static final String noteToScoreTemplate = "Note %s Score %s";
    private static final String scoreToNoteTemplate = "Score %s Note %s";
    
    private static final RobotState robotState = RobotState.getInstance();
    
    public ScoreNote(Drive drive, Intake intake, Pivot pivot, Shooter shooter, Kicker kicker) {
        super(
            "ScoreNote",
            MAX_QUESTION_COUNT,
            () -> {
                int requiredNotes = numberNotes.getResponse().ordinal(); // selected number of notes

                if (requiredNotes < notes.size()) {
                    for (int i = notes.size() - 1; i >= requiredNotes; i--) {
                        notes.remove(i);
                        scorePositions.remove(i);
                    }
                } else if (requiredNotes > notes.size()) {             
                    for (int i = notes.size(); i < requiredNotes; i++) {
                        final int idx = i;
                        AutoQuestion<Note> note = new AutoQuestion<>(
                            "Note #" + Integer.toString(i + 1),
                            () -> {
                                List<Note> noteOptions = Arrays.asList(Note.values());
                                List<Note> filteredNotes = new ArrayList<>();
                                List<Note> visitedNotes = new ArrayList<>();
                                for (int k = 0; k < idx; k++) {
                                    visitedNotes.add(notes.get(k).getResponse());
                                }                                
                                for (Note noteOption : noteOptions) {
                                    if (!visitedNotes.contains(noteOption)) {
                                      filteredNotes.add(noteOption);
                                    }
                                }
                                return filteredNotes.toArray(new Note[0]);
                            }
                        );
                        AutoQuestion<ScorePosition> scorePosition = new AutoQuestion<>("Note #" + Integer.toString(i + 1) + " Scoring Position", ScorePosition::values);
                        notes.add(note);
                        scorePositions.add(scorePosition);
                    }
                }

                List<AutoQuestion<?>> a = new ArrayList<>();
                a.add(startPosition);
                a.add(numberNotes);
                for (int i = 0; i < notes.size(); i++) {
                    a.add(notes.get(i));
                    a.add(scorePositions.get(i));
                }
                return a;
            },
            () -> {
                Function<PathPlannerPath, Command> followPathConstructor = (path) -> new FollowPathHolonomic(path, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, Drive.autoConfigSup.get(), AllianceFlipUtil::shouldFlip, drive);

                if (notes.size() == 0) {
                    PathPlannerPath startDrive = PathPlannerPath.fromPathFile(String.format(startDriveTemplate, startPosition.getResponse().toString()));
                    return Commands.runOnce(() -> robotState.setPose(drive.getGyroRotation(), drive.getModulePositions(), AllianceFlipUtil.apply(new Pose2d(startDrive.getPoint(0).position, new Rotation2d()))));
                } else {
                    PathPlannerPath startToNote = PathPlannerPath.fromPathFile(String.format(startToNoteTemplate, startPosition.getResponse().toString(), notes.get(0).getResponse().ordinal() + 1));
                    PathPlannerPath noteToScore = PathPlannerPath.fromPathFile(String.format(noteToScoreTemplate, notes.get(0).getResponse().ordinal() + 1, scorePositions.get(0).getResponse().toString()));

                    List<PathPlannerPath> notePaths = new ArrayList<>();
                    notePaths.add(startToNote);
                    notePaths.add(noteToScore);
                    for (int i = 1; i < notes.size(); i++) {
                        PathPlannerPath newNoteToScore = PathPlannerPath.fromPathFile(String.format(noteToScoreTemplate, notes.get(i).getResponse().ordinal() + 1, scorePositions.get(i).toString()));
                        notePaths.add(newNoteToScore);
                        if (scorePositions.get(i - 1).equals(ScorePosition.Existing)) {
                            PathPlannerPath noteToNewNote = PathPlannerPath.fromPathFile(String.format(noteToNoteTemplate, notes.get(i - 1).getResponse().ordinal() + 1, notes.get(i).getResponse().ordinal() + 1));
                            notePaths.add(noteToNewNote);
                        } else {
                            PathPlannerPath scoringToNewNote = PathPlannerPath.fromPathFile(String.format(scoreToNoteTemplate, scorePositions.get(i - 1).toString(), notes.get(i).getResponse().ordinal() + 1));
                            notePaths.add(scoringToNewNote);
                        }
                    }

                    Supplier<Translation2d> shootAtPos = () -> {
                        var speakerTrans = AllianceFlipUtil.apply(FieldConstants.speakerCenter);
                        var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(drive.getChassisSpeeds(), drive.getPose().getRotation());
                        var robotToSpeaker = speakerTrans.minus(drive.getPose().getTranslation());
                        var robotToSpeakerNorm = robotToSpeaker.div(robotToSpeaker.getNorm());
                        var velocityTowardsSpeaker = MathExtraUtil.dotProduct(robotToSpeakerNorm, new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
                        var timeToSpeaker = drive.getPose().getTranslation().getDistance(speakerTrans) / (ShooterConstants.exitVelocity + velocityTowardsSpeaker);
                        var chassisOffset = chassisSpeeds.times(timeToSpeaker);
                        var translationalOffset = new Translation2d(chassisOffset.vxMetersPerSecond, chassisOffset.vyMetersPerSecond);
                        var pointTo = speakerTrans.minus(translationalOffset);
                        Logger.recordOutput("Shooter/Shoot At", pointTo);
                        return pointTo;
                    };

                    List<Command> commands = new ArrayList<>();
                    for (int i = 0; i < notePaths.size(); i++) {
                        PathPlannerPath notePath = notePaths.get(i);

                        if (i % 2 == 0) {
                            commands.add(followPathConstructor.apply(notePath).alongWith(intake.intake(drive::getChassisSpeeds)));
                        } else {
                            commands.add(followPathConstructor.apply(notePath).alongWith(shooter.shoot(shootAtPos), pivot.autoAim(shootAtPos)).andThen(kicker.feedIn()));
                        }
                    }

                    return Commands.runOnce(() -> robotState.setPose(drive.getGyroRotation(), drive.getModulePositions(), AllianceFlipUtil.apply(new Pose2d(startToNote.getPoint(0).position, new Rotation2d())))).andThen(commands.toArray(new Command[0]));
                }
            }
        );
    }
}