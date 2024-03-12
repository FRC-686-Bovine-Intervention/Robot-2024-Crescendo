package frc.robot.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class TunableFeedForward {
    public SimpleMotorFeedforward ff;

    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;
    private final LoggedTunableNumber kS;

    public TunableFeedForward(String key, double kV, double kA, double kS) {
        this.kV = new LoggedTunableNumber(key + "/kV", kV);
        this.kA = new LoggedTunableNumber(key + "/kA", kA);
        this.kS = new LoggedTunableNumber(key + "/kS", kS);
        update();
    }

    public void update() {
        if(kV.hasChanged(hashCode()) | kA.hasChanged(hashCode()) | kS.hasChanged(hashCode())) {
            ff = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
        }
    }
}
