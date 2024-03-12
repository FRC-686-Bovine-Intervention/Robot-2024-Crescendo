package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class TunableProfiledPID {
    private final ProfiledPIDController pid;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    public TunableProfiledPID(String key, ProfiledPIDController pid) {
        this.pid = pid;
        this.kP = new LoggedTunableNumber(key + "/kP", pid.getP());
        this.kI = new LoggedTunableNumber(key + "/kI", pid.getI());
        this.kD = new LoggedTunableNumber(key + "/kD", pid.getD());
        this.kV = new LoggedTunableNumber(key + "/kV", pid.getConstraints().maxVelocity);
        this.kA = new LoggedTunableNumber(key + "/kA", pid.getConstraints().maxAcceleration);
    }

    public void update() {
        if(kP.hasChanged(hashCode()) | kI.hasChanged(hashCode()) | kD.hasChanged(hashCode())) {
            pid.setPID(kP.get(), kI.get(), kD.get());
        }
        if(kV.hasChanged(hashCode()) | kA.hasChanged(hashCode())) {
            pid.setConstraints(new Constraints(kV.get(), kA.get()));
        }
    }
}
