// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise.
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall. Use the {@link #allianceFlip(Translation2d)} and {@link #allianceFlip(Pose2d)}
 * methods to flip these values based on the current alliance color.
 */
public final class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(648);
  public static final double fieldWidth =  Units.inchesToMeters(324);

  public static final Translation2d speakerCenter = new Translation2d(0.240581, 5.547755);

  public static final Pose2d ampFront = new Pose2d(new Translation2d(1.83, 7.61), new Rotation2d(Degrees.of(90).in(Radians)));
  public static final Pose2d speakerFront = new Pose2d(new Translation2d(1.40, 5.55), new Rotation2d(Degrees.of(180).in(Radians)));
  public static final Pose2d sourceFront = new Pose2d(new Translation2d(15.41, 1.04), new Rotation2d(Degrees.of(-60).in(Radians)));
  public static final Pose2d podiumFront = new Pose2d(new Translation2d(2.54, 4.12), new Rotation2d(Degrees.of(180).in(Radians)));
}
