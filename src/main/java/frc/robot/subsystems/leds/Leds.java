package frc.robot.subsystems.leds;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotType;
import frc.robot.Constants.CANDevices;
import frc.robot.RobotType.Mode;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.led.animation.EndgameTimerAnimation;
import frc.robot.util.led.animation.FillAnimation;
import frc.robot.util.led.animation.LEDAnimation;
import frc.robot.util.led.animation.LEDManager;
import frc.robot.util.led.animation.ScrollingAnimation;
import frc.robot.util.led.functions.Gradient.BasicGradient;
import frc.robot.util.led.functions.Gradient.BasicGradient.InterpolationStyle;
import frc.robot.util.led.functions.TilingFunction;
import frc.robot.util.led.strips.LEDStrip;
import frc.robot.util.led.strips.hardware.CANdleStrip;

public class Leds extends VirtualSubsystem {
    private final LEDManager ledManager = LEDManager.getInstance();
    @SuppressWarnings("unused")
    private final LEDStrip onboardLEDs;
    private final LEDStrip offboardLEDs;

    private final AnimationRunner[] runners;

    public Leds(
        BooleanSupplier robotAutonomous, 
        BooleanSupplier intaking, BooleanSupplier intakeReversed, BooleanSupplier intakeSecuring, BooleanSupplier intakeSecured, 
        BooleanSupplier kickerFeeding, BooleanSupplier kickerLoaded
    ) {
        System.out.println("[Init Leds] Instantiating Leds");
        if(RobotType.getMode() == Mode.REAL) {
            var m_candle = new CANdle(CANDevices.candleCanID, "rio");
            var candleStrip = new CANdleStrip(m_candle, 60*2);

            ledManager.register(candleStrip);

            onboardLEDs = candleStrip.getOnboardLEDs();
            offboardLEDs = candleStrip.getOffboardLEDs();
            
            CANdleConfiguration configAll = new CANdleConfiguration();
            configAll.statusLedOffWhenActive = true;
            configAll.disableWhenLOS = false;
            configAll.stripType = LEDStripType.GRB;
            configAll.brightnessScalar = 0.5;
            configAll.vBatOutputMode = VBatOutputMode.Modulated;
            m_candle.configFactoryDefault();
            m_candle.clearAnimation(0);
            m_candle.configAllSettings(configAll, 100);
        } else {
            onboardLEDs = new LEDStrip() {
                @Override
                public int getLength() {
                    return 0;
                }
                @Override
                public void setLED(int ledIndex, Color color) {
                }
            };
            offboardLEDs = new LEDStrip() {
                @Override
                public int getLength() {
                    return 0;
                }
                @Override
                public void setLED(int ledIndex, Color color) {
                }
            };
        }
        

        // Assuming ccw circular pattern
        var shooterStrip = offboardLEDs.substrip(0, 12);
        var rightStrip = offboardLEDs.substrip(12, 24);
        var frontStrip = offboardLEDs.substrip(24, 36);
        var leftStrip = offboardLEDs.substrip(36, 48);

        var sideStrips = leftStrip.reverse().parallel(rightStrip);

        var leftRingStrip = frontStrip.substrip(6).concat(leftStrip).concat(shooterStrip.substrip(0, 6)).reverse();
        var rightRingStrip = shooterStrip.substrip(6).concat(rightStrip).concat(frontStrip.substrip(0, 6));
        var sideRingStrips = leftRingStrip.parallel(rightRingStrip);

        this.runners = new AnimationRunner[]{
            new AnimationRunner(
                "Endgame Timer",
                () -> DriverStation.getMatchType() != MatchType.None && DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() <= 30,
                new EndgameTimerAnimation(
                    10,
                    offboardLEDs
                )
            ),
            new AnimationRunner(
                "Autonomous Robot",
                robotAutonomous,
                new ScrollingAnimation(
                    4,
                    new BasicGradient(InterpolationStyle.Step, Color.kRed, Color.kBlue),
                    TilingFunction.Modulo,
                    2,
                    4,
                    offboardLEDs
                )
            ),
            new AnimationRunner(
                "DriverStation Connection",
                DriverStation::isDisabled,
                new FillAnimation(
                    1,
                    () -> (DriverStation.isDSAttached() ? Color.kGreen : Color.kOrange),
                    offboardLEDs.substrip(0, 2)
                )
            ),
            // Intake
            new AnimationRunner(
                "Intaking Forward",
                () -> !intakeReversed.getAsBoolean() && (intaking.getAsBoolean() || intakeSecuring.getAsBoolean() || kickerFeeding.getAsBoolean()), 
                new ScrollingAnimation(
                    0,
                    (x) -> InterpolationStyle.Linear.interpolate(x, intakeSecuring.getAsBoolean() ? new Color[]{Color.kGreen, Color.kYellow} : new Color[]{Color.kRed, Color.kYellow}),
                    TilingFunction.Sinusoidal,
                    5,
                    2,
                    sideRingStrips
                )
            ),
            new AnimationRunner(
                "Intaking Reversed",
                () -> intakeReversed.getAsBoolean() && (intaking.getAsBoolean() || intakeSecuring.getAsBoolean()), 
                new ScrollingAnimation(
                    0,
                    (x) -> InterpolationStyle.Linear.interpolate(x, intakeSecuring.getAsBoolean() ? new Color[]{Color.kGreen, Color.kYellow} : new Color[]{Color.kRed, Color.kYellow}),
                    TilingFunction.Sinusoidal,
                    -5,
                    2,
                    sideRingStrips
                )
            ),
            new AnimationRunner(
                "Intake Secured",
                intakeSecured, 
                new FillAnimation(
                    0, 
                    Color.kGreen,
                    sideRingStrips
                )
            ),
            // Kicker
            new AnimationRunner(
                "Kicker Loaded",
                kickerLoaded, 
                new FillAnimation(
                    0, 
                    Color.kGreen, 
                    sideRingStrips
                )
            ),
        };

        new ScrollingAnimation(
            0,
            (x) -> {
                var colors = new Color[]{
                    (DriverStation.getAlliance().isEmpty() ? Color.kRed : Color.kBlack),
                    (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? Color.kRed : Color.kFirstBlue)
                };
                return InterpolationStyle.Linear.interpolate(x, colors);
            },
            TilingFunction.Sinusoidal,
            1,
            4,
            offboardLEDs
        ).start();
    }

    @Override
    public void periodic() {
        Arrays.stream(runners).forEach(AnimationRunner::update);
        ledManager.runLEDs();
    }

    public static class AnimationRunner {
        private final String key;
        private final BooleanSupplier runAnimationSupplier;
        private final LEDAnimation animation;
        private boolean lastVal;

        public AnimationRunner(String key, BooleanSupplier runAnimationSupplier, LEDAnimation animation) {
            this.key = key;
            this.runAnimationSupplier = runAnimationSupplier;
            this.animation = animation;
        }

        public void update() {
            var curVal = runAnimationSupplier.getAsBoolean();
            SmartDashboard.putBoolean("LEDs/"+key, curVal);
            if(curVal ^ lastVal) {
                if(curVal) {
                    animation.start();
                } else {
                    animation.pause();
                }
            }
            lastVal = curVal;
        }
    }
}
