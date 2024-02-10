package frc.robot.subsystems.drive.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.function.ToDoubleBiFunction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AllianceFlipUtil.FieldFlipType;
import frc.robot.util.controllers.Joystick;

public class DriveWithCustomFlick extends Command {

	private final Drive drive;
    private final Supplier<ChassisSpeeds> fieldRelativeSupplier;
	private final ToDoubleBiFunction<Rotation2d, Optional<Rotation2d>> headingToTurnRate;
	private Optional<Rotation2d> override;

	public static Supplier<Optional<Rotation2d>> headingFromJoystick(Joystick joystick, Supplier<Rotation2d[]> snapPointsSupplier, Supplier<Rotation2d> forwardDirectionSupplier) {
		return new Supplier<Optional<Rotation2d>>() {
			private final Timer preciseTurnTimer = new Timer();
			private final double preciseTurnTimeThreshold = 0.5;
			@Override
			public Optional<Rotation2d> get() {
				Function<Rotation2d, Optional<Rotation2d>> outputFilter = (i) -> Optional.of(i.minus(forwardDirectionSupplier.get()));
				if(joystick.magnitude() == 0) {
					preciseTurnTimer.restart();
					return Optional.empty();
				}
				Rotation2d joyHeading = AllianceFlipUtil.apply(Rotation2d.fromRadians(joystick.radsFromPosYCCW()), FieldFlipType.CenterPointFlip);
				if(preciseTurnTimer.hasElapsed(preciseTurnTimeThreshold)) {
					return outputFilter.apply(joyHeading);
				}
				var snapPoints = snapPointsSupplier.get();
				int smallestDistanceIndex = 0;
				double smallestDistance = Double.MAX_VALUE;
				for(int i = 0; i < snapPoints.length; i++) {
					var dist = Math.abs(joyHeading.minus(AllianceFlipUtil.apply(snapPoints[i])).getRadians());
					if(dist < smallestDistance) {
						smallestDistance = dist;
						smallestDistanceIndex = i;
					}
				}
				return outputFilter.apply(AllianceFlipUtil.apply(snapPoints[smallestDistanceIndex]));
			}
		};
	}

	public static Supplier<Optional<Rotation2d>> pointTo(Supplier<Translation2d> posToPointTo) {
        return () -> {
            var pointTo = posToPointTo.get();
            var desiredHeading = Rotation2d.fromRadians(Math.atan2(
                pointTo.getY() - RobotState.getInstance().getPose().getY(),
                pointTo.getX() - RobotState.getInstance().getPose().getX()
            ));
            return Optional.of(desiredHeading);
        };
    }

	public static ToDoubleBiFunction<Rotation2d, Optional<Rotation2d>> pidControlledHeading(Supplier<Optional<Rotation2d>> headingSupplier) {
		return new ToDoubleBiFunction<Rotation2d, Optional<Rotation2d>>() {
			private final PIDController headingPID = new PIDController(DriveConstants.headingKp, DriveConstants.headingKi, DriveConstants.headingKd);
			{
				headingPID.enableContinuousInput(-Math.PI, Math.PI);  // since gyro angle is not limited to [-pi, pi]
				headingPID.setTolerance(DriveConstants.headingTolerance);
			}
			private Rotation2d desiredHeading;
			@Override
			public double applyAsDouble(Rotation2d robotHeading, Optional<Rotation2d> override) {
				override.ifPresent((r) -> desiredHeading = r);
				headingSupplier.get().ifPresent((r) -> desiredHeading = r);
				double turnInput = headingPID.calculate(robotHeading.getRadians(), desiredHeading.getRadians());
				turnInput = headingPID.atSetpoint() ? 0 : turnInput;
				turnInput = MathUtil.clamp(turnInput, -0.5, +0.5);
				return turnInput * DriveConstants.maxTurnRateRadiansPerSec;
			}
		};
	}

	public static Supplier<ChassisSpeeds> joystickControlledFieldRelative(Joystick translationalJoystick, BooleanSupplier precisionSupplier) {
		return () -> AllianceFlipUtil.applyFieldRelative(
			new ChassisSpeeds(
				translationalJoystick.y().getAsDouble()	 * DriveConstants.maxDriveSpeedMetersPerSec * (precisionSupplier.getAsBoolean() ? DriveConstants.precisionLinearMultiplier : 1),
				-translationalJoystick.x().getAsDouble() * DriveConstants.maxDriveSpeedMetersPerSec * (precisionSupplier.getAsBoolean() ? DriveConstants.precisionLinearMultiplier : 1),
				0
			),
			FieldFlipType.CenterPointFlip
		);
	}

	public DriveWithCustomFlick(Drive drive, Supplier<ChassisSpeeds> fieldRelativeSupplier, ToDoubleBiFunction<Rotation2d, Optional<Rotation2d>> headingToTurnRate) {
		addRequirements(drive);
		setName("DriveWithCustomFlick");
		this.drive = drive;
        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.headingToTurnRate = headingToTurnRate;
	}

	@Override
	public void initialize() {
		override = Optional.of(drive.getPose().getRotation());
	}

	@Override
	public void execute() {
		var speeds = fieldRelativeSupplier.get();
		speeds.omegaRadiansPerSecond = headingToTurnRate.applyAsDouble(drive.getPose().getRotation(), override);
		override = Optional.empty();

		speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getPose().getRotation());

		drive.driveVelocity(speeds);
	}

	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}
}
