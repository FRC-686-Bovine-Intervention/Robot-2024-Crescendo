package frc.robot.subsystems.drive.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AllianceFlipUtil.FieldFlipType;
import frc.robot.util.controllers.Joystick;

public class AutoAim extends Command {
    private final Drive drive;
    private final Joystick translationalJoystick;
    private final BooleanSupplier precisionSupplier;
    private final Supplier<Translation2d> posToPointTo;

    private final double headingKp = 0.3 /* / DriveConstants.maxTurnRateRadiansPerSec */;
	private final double headingKi = 0;
	private final double headingKd = 0;
	private final double headingTolerance = Units.degreesToRadians(1.0);
	private final PIDController headingPID;

    public AutoAim(Drive drive, Joystick translationalJoystick, BooleanSupplier precisionSupplier, Supplier<Translation2d> posToPointTo) {
        addRequirements(drive);
        setName("Auto Aim");
        this.drive = drive;
        this.translationalJoystick = translationalJoystick;
        this.precisionSupplier = precisionSupplier;
        this.posToPointTo = posToPointTo;

        headingPID = new PIDController(headingKp, headingKd, headingKi);
		headingPID.enableContinuousInput(-Math.PI, Math.PI);  // since gyro angle is not limited to [-pi, pi]
		headingPID.setTolerance(headingTolerance);
    }

    @Override
    public void execute() {
        var pointTo = posToPointTo.get();
        var desiredHeading = Rotation2d.fromRadians(Math.atan2(
            pointTo.getY() - drive.getPose().getY(),
            pointTo.getX() - drive.getPose().getX()
        ));
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
}
