package frc.robot.subsystems.drive.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AllianceFlipUtil.FieldFlipType;
import frc.robot.util.controllers.Joystick;

public class DriveWithCustomFlick extends Command {

	private final Drive drive;
    private final Joystick translationalJoystick;
	private final Supplier<Optional<Rotation2d>> headingSupplier; // rotation
	private final BooleanSupplier precisionSupplier; // slow-down for precision positioning

	private Rotation2d desiredHeading;
	private final double headingKp = 0.3 /* / DriveConstants.maxTurnRateRadiansPerSec */;
	private final double headingKi = 0;
	private final double headingKd = 0;
	private final double headingTolerance = Units.degreesToRadians(1.0);
	private final PIDController headingPID;

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

	public DriveWithCustomFlick(Drive drive, Joystick translationalJoystick, Supplier<Optional<Rotation2d>> headingSupplier, BooleanSupplier precisionSupplier) {
		addRequirements(drive);
		setName("DriveWithCustomFlick");
		this.drive = drive;
        this.translationalJoystick = translationalJoystick;
        this.headingSupplier = headingSupplier;
		this.precisionSupplier = precisionSupplier;

		headingPID = new PIDController(headingKp, headingKd, headingKi);
		headingPID.enableContinuousInput(-Math.PI, Math.PI);  // since gyro angle is not limited to [-pi, pi]
		headingPID.setTolerance(headingTolerance);
	}

	@Override
	public void initialize() {
		desiredHeading = drive.getPose().getRotation();
	}

	@Override
	public void execute() {
		// update desired direction
		Optional<Rotation2d> optDesiredHeading = headingSupplier.get();
		if(optDesiredHeading.isPresent()) {
			desiredHeading = optDesiredHeading.get();
		}

		// PID control of turn
		double turnInput = headingPID.calculate(drive.getPose().getRotation().getRadians(), desiredHeading.getRadians());
		turnInput = headingPID.atSetpoint() ? 0 : turnInput;
		turnInput = MathUtil.clamp(turnInput, -0.5, +0.5);

        // Convert to meters/sec and radians/sec
        double vxMetersPerSecond = translationalJoystick.y().getAsDouble() * drive.getMaxLinearSpeedMetersPerSec();
        double vyMetersPerSecond = -translationalJoystick.x().getAsDouble() * drive.getMaxLinearSpeedMetersPerSec();
        double omegaRadiansPerSecond = turnInput * drive.getMaxAngularSpeedRadiansPerSec();

        if(precisionSupplier.getAsBoolean()) {
            vxMetersPerSecond *= DriveConstants.precisionLinearMultiplier;
            vyMetersPerSecond *= DriveConstants.precisionLinearMultiplier;
            omegaRadiansPerSecond *= DriveConstants.precisionTurnMulitiplier;
        }

		// robot relative controls
		ChassisSpeeds speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

		// field relative controls
		var driveRotation = AllianceFlipUtil.apply(drive.getRotation(), FieldFlipType.CenterPointFlip);
		speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, driveRotation);

		drive.driveVelocity(speeds);
	}

	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}
}
