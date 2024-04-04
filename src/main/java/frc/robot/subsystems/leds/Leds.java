package frc.robot.subsystems.leds;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotType;
import frc.robot.RobotType.Mode;
import frc.robot.subsystems.climber.Climber;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.led.animation.EndgameTimerAnimation;
import frc.robot.util.led.animation.FillAnimation;
import frc.robot.util.led.animation.FlashingAnimation;
import frc.robot.util.led.animation.LEDAnimation;
import frc.robot.util.led.animation.LEDManager;
import frc.robot.util.led.animation.ScrollingAnimation;
import frc.robot.util.led.functions.Gradient.BasicGradient;
import frc.robot.util.led.functions.Gradient.BasicGradient.InterpolationStyle;
import frc.robot.util.led.functions.TilingFunction;
import frc.robot.util.led.strips.LEDStrip;
import frc.robot.util.led.strips.hardware.AddressableStrip;

public class Leds extends VirtualSubsystem {
    private final LEDManager ledManager = LEDManager.getInstance();
    @SuppressWarnings("unused")
    // private final LEDStrip onboardLEDs;
    private final LEDStrip offboardLEDs;
    private final LEDStrip rightStrip;
    private final LEDStrip backStrip;
    private final LEDStrip leftStrip;

    private final LEDStrip sideStrips;
    private final LEDStrip sideStripTips;

    private final LEDStrip backRightStrip;
    private final LEDStrip backLeftStrip;

    private final LEDStrip fullLeftStrip;
    private final LEDStrip fullRightStrip;

    private final LEDStrip fullSideStrips;

    private final LEDStrip backMirroredStrip;

    public Leds() {
        System.out.println("[Init Leds] Instantiating Leds");
        if(RobotType.getMode() == Mode.REAL) {
            var addressableStrip = new AddressableStrip(0, 57);
            ledManager.register(addressableStrip);
            offboardLEDs = addressableStrip;
        } else {
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

        rightStrip = offboardLEDs.substrip(0, 19);
        backStrip = offboardLEDs.substrip(19, 38);
        leftStrip = offboardLEDs.substrip(38, 57).reverse();

        sideStrips = leftStrip.parallel(rightStrip);
        sideStripTips = sideStrips.substrip(15).concat(backStrip.substrip(5, 13));
        
        backRightStrip = backStrip.substrip(0, 10);
        backLeftStrip = backStrip.substrip(9).reverse();
        
        fullLeftStrip = leftStrip.concat(backLeftStrip);
        fullRightStrip = rightStrip.concat(backRightStrip);
        
        fullSideStrips = fullLeftStrip.parallel(fullRightStrip);
        
        backMirroredStrip = backRightStrip.reverse().parallel(backLeftStrip.reverse());

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
            fullSideStrips
        ).schedule();

        new Trigger(() -> DriverStation.getMatchType() != MatchType.None && DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() <= 30)
        .whileTrue(
            new EndgameTimerAnimation(
                4,
                sideStrips
            )
        ).whileTrue(
            new EndgameTimerAnimation(
                4,
                backMirroredStrip
            )
        );

        new Trigger(DriverStation::isDisabled).debounce(1)
        .whileTrue(
            new FillAnimation(
                1,
                () -> (DriverStation.isDSAttached() ? Color.kGreen : Color.kOrange),
                sideStrips.substrip(0, 2)
            )
        );
    }

    @Override
    public void periodic() {
        ledManager.run();
    }

    public Command noteAcquired() {
        return new FlashingAnimation(
            20,
            new BasicGradient(InterpolationStyle.Linear, Color.kBlack, Color.kGreen),
            TilingFunction.Sawtooth,
            fullSideStrips
        ).setPeriod(0.125).withTimeout(1);
    }

    public Command noteSecured() {
        return new FillAnimation(
            5,
            Color.kLime,
            sideStripTips
        );
    }

    public Command visionAcquired() {
        return new FillAnimation(
            3,
            Color.kOrange,
            sideStripTips
        );
    }

    public Command visionLocked() {
        return new FillAnimation(
            3,
            Color.kPurple,
            sideStripTips
        );
    }

    public Command humanPlayerFlash() {
        return new FlashingAnimation(
            15,
            new BasicGradient(InterpolationStyle.Step, Color.kBlack, Color.kWhite),
            TilingFunction.Sawtooth,
            fullSideStrips
        ).setPeriod(0.125).withTimeout(1);
    }

    public Command shooterBarGraph(DoubleSupplier shooterSpeed, DoubleSupplier shooterTarget, BooleanSupplier shooterReady) {
        return new LEDAnimation(12) {
            @Override
            public void execute() {
                sideStrips.foreach((i) -> {
                    var pos = (double) i / sideStrips.getLength();
                    var barPos = Math.sqrt(shooterSpeed.getAsDouble() / 30);
                    sideStrips.setLED(i, (pos <= barPos ? (shooterReady.getAsBoolean() ? Color.kGreen : Color.kRed) : (shooterReady.getAsBoolean() ? new Color(0, 0.03, 0) : Color.kBlack)));
                });
                var dotPos = (int)Math.ceil(Math.sqrt(shooterTarget.getAsDouble() / 30) * (sideStrips.getLength() - 1));
                sideStrips.setLED(dotPos, Color.kGreen);
            }
        };
    }

    public Command defenseSpinActivated() {
        return new FlashingAnimation(
            10,
            new BasicGradient(InterpolationStyle.Linear, Color.kBlack, Color.kYellow),
            TilingFunction.Sinusoidal,
            fullSideStrips
        ).setPeriod(0.25);
    }

    public Command climbingModeActivated() {
        return new FlashingAnimation(
            6,
            new BasicGradient(InterpolationStyle.Linear, Color.kBlack, Color.kTeal),
            TilingFunction.Sinusoidal,
            fullSideStrips
        ).setPeriod(0.75);
    }

    public Command climbing(DoubleSupplier climbingPos) {
        return new LEDAnimation(25) {
            @Override
            public void execute() {
                sideStrips.foreach((i) -> {
                    var pos = (double) i / sideStrips.getLength();
                    var barPos = 1 - (climbingPos.getAsDouble() / Climber.POS_DEPLOY);
                    sideStrips.setLED(i, (pos <= barPos ? Color.kTeal : Color.kBlack));
                });
                backMirroredStrip.foreach((i) -> {
                    var pos = (double) i / backMirroredStrip.getLength();
                    var barPos = 1 - (climbingPos.getAsDouble() / Climber.POS_DEPLOY);
                    backMirroredStrip.setLED(i, (pos <= barPos ? Color.kTeal : Color.kBlack));
                });
            }
        };
    }

    public LEDStrip getLeftApriltagStrip() {
        return sideStrips.substrip(2,3);
    }

    public LEDStrip getRightApriltagStrip() {
        return sideStrips.substrip(3,4);
    }

    public LEDStrip getNoteVisionStrip() {
        return sideStrips.substrip(4,5);
    }
}
