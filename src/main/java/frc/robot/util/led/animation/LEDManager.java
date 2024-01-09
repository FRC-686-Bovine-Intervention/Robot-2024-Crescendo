package frc.robot.util.led.animation;

import java.util.ArrayList;

import frc.robot.util.led.strips.hardware.HardwareStrip;

public class LEDManager {
    private static LEDManager instance;
    public static LEDManager getInstance() {if(instance == null) instance = new LEDManager(); return instance;}

    private final ArrayList<LEDAnimation> runningAnimations = new ArrayList<>();
    private final ArrayList<HardwareStrip> registeredStrips = new ArrayList<>();

    public void runLEDs() {
        runningAnimations.stream()
                         .sorted((LEDAnimation a, LEDAnimation b) -> (a.getPriority() - b.getPriority()))
                         .forEachOrdered((LEDAnimation anim) -> anim.runAnimation(this));
        registeredStrips.forEach((HardwareStrip s) -> s.refresh());
    }

    public boolean isAnimationRunning(LEDAnimation animation) {
        return runningAnimations.contains(animation);
    }

    public boolean play(LEDAnimation animation) {
        if(!isAnimationRunning(animation)) {
            runningAnimations.add(animation);
            animation.animationTimer.start();
            return true;
        }
        return false;
    }

    public boolean pause(LEDAnimation animation) {
        if(isAnimationRunning(animation)) {
            runningAnimations.remove(animation);
            animation.animationTimer.stop();
            return true;
        }
        return false;
    }

    public boolean stop(LEDAnimation animation) {
        if(isAnimationRunning(animation)) {
            runningAnimations.remove(animation);
            animation.animationTimer.stop();
            animation.animationTimer.reset();
            return true;
        }
        return false;
    }

    public boolean stopAll() {
        for (var animation : runningAnimations) {
            stop(animation);
        }
        return runningAnimations.isEmpty();
    }

    public LEDManager register(HardwareStrip... hardwareStrips) {
        for(HardwareStrip hardwareStrip : hardwareStrips) {
            if(!registeredStrips.contains(hardwareStrip)) {
                registeredStrips.add(hardwareStrip);
            }
        }
        return this;
    }
}
