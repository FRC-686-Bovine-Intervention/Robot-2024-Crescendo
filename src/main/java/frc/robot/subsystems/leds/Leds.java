package frc.robot.subsystems.leds;

import java.util.Optional;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.CANDevices;
import frc.robot.RobotType;
import frc.robot.RobotType.Mode;
import frc.robot.util.InterpolationFunction;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.led.animation.FlashingAnimation;
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

    public Leds() {
        System.out.println("[Init Leds] Instantiating Leds");
        if(RobotType.getMode() == Mode.REAL) {
            var m_candle = new CANdle(CANDevices.candleCanID, "rio");
            var candleStrip = new CANdleStrip(m_candle, 57);

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
            m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 2000);
            m_candle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, 1000);
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

        var rightStrip = offboardLEDs.substrip(0, 19);
        var backStrip = offboardLEDs.substrip(19, 38);
        var leftStrip = offboardLEDs.substrip(38, 57).reverse();

        var sideStrips = leftStrip.parallel(rightStrip);

        var backRightStrip = backStrip.substrip(0, 10);
        var backLeftStrip = backStrip.substrip(10, 19).reverse();

        var fullLeftStrip = leftStrip.concat(backLeftStrip);
        var fullRightStrip = rightStrip.concat(backRightStrip);

        var fullSideStrips = fullLeftStrip.parallel(fullRightStrip);
        
        // this.runners = new AnimationRunner[]{
        //     // new AnimationRunner(
        //     //     "Endgame Timer",
        //     //     () -> DriverStation.getMatchType() != MatchType.None && DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() <= 30,
        //     //     new EndgameTimerAnimation(
        //     //         10,
        //     //         offboardLEDs
        //     //     )
        //     // ),
        //     // new AnimationRunner(
        //     //     "Autonomous Robot",
        //     //     robotAutonomous,
        //     //     new ScrollingAnimation(
        //     //         4,
        //     //         new BasicGradient(InterpolationStyle.Step, Color.kRed, Color.kBlue),
        //     //         TilingFunction.Modulo,
        //     //         2,
        //     //         4,
        //     //         offboardLEDs
        //     //     )
        //     // ),
        //     new AnimationRunner(
        //         "DriverStation Connection",
        //         DriverStation::isDisabled,
        //         new FillAnimation(
        //             1,
        //             () -> (DriverStation.isDSAttached() ? Color.kGreen : Color.kOrange),
        //             sideStrips.substrip(0, 2)
        //         )
        //     ),
        //     // Intake
        //     new AnimationRunner(
        //         "Intaking Forward",
        //         () -> !intakeReversed.getAsBoolean() && (intaking.getAsBoolean() || kickerFeeding.getAsBoolean()), 
        //         new ScrollingAnimation(
        //             0,
        //             (x) -> InterpolationStyle.Linear.interpolate(x, kickerFeeding.getAsBoolean() ? new Color[]{Color.kGreen, Color.kYellow} : new Color[]{Color.kRed, Color.kYellow}),
        //             TilingFunction.Sinusoidal,
        //             2,
        //             2,
        //             fullSideStrips
        //         )
        //     ),
        //     new AnimationRunner(
        //         "Intaking Reversed",
        //         () -> intakeReversed.getAsBoolean() && (intaking.getAsBoolean() || kickerFeeding.getAsBoolean()), 
        //         new ScrollingAnimation(
        //             0,
        //             (x) -> InterpolationStyle.Linear.interpolate(x, kickerFeeding.getAsBoolean() ? new Color[]{Color.kGreen, Color.kYellow} : new Color[]{Color.kRed, Color.kYellow}),
        //             TilingFunction.Sinusoidal,
        //             -2,
        //             2,
        //             fullSideStrips
        //         )
        //     ),
        //     // Kicker
        //     new AnimationRunner(
        //         "Kicker Loaded",
        //         kickerLoaded, 
        //         new FillAnimation(
        //             0, 
        //             Color.kGreen, 
        //             fullSideStrips
        //         )
        //     ),
        // };

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
    }

    @Override
    public void periodic() {
        ledManager.run();
    }

    public LEDAnimation noteAcquired() {
        return new FlashingAnimation(
            5,
            new BasicGradient(InterpolationStyle.Step, Color.kBlack, Color.kGreen),
            TilingFunction.Sawtooth,
            offboardLEDs
        );
    }
}
