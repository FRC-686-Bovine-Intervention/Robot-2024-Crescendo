package frc.robot.util.controllers;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.controllers.Joystick.Axis;

public class Thrustmaster {
    private final edu.wpi.first.wpilibj.Joystick hid;
    public final Joystick joystick;
    public final Axis twist;
    public final Axis slider;

    public Thrustmaster(int port) {
        hid = new edu.wpi.first.wpilibj.Joystick(port);

        joystick = new Joystick(hid::getX, hid::getY).invertY();
        twist = new Axis(hid::getTwist).invert();
        slider = new Axis(hid::getThrottle);
    }

    public Trigger leftTopLeft() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(5));}
    public Trigger leftTopCenter() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(6));}
    public Trigger leftTopRight() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(7));}
    public Trigger leftBottomLeft() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(10));}
    public Trigger leftBottomCenter() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(9));}
    public Trigger leftBottomRight() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(8));}

    public Trigger rightTopLeft() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(13));}
    public Trigger rightTopCenter() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(12));}
    public Trigger rightTopRight() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(11));}
    public Trigger rightBottomLeft() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(14));}
    public Trigger rightBottomCenter() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(15));}
    public Trigger rightBottomRight() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(16));}

    public Trigger stickTrigger() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(1));}
    public Trigger stickLeft() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(3));}
    public Trigger stickRight() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(4));}
    public Trigger stickBottom() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(2));}

    public Trigger povCenter() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getPOV() == -1);}
    public Trigger povUp() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getPOV() == 0);}
    public Trigger povUpRight() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getPOV() == 45);}
    public Trigger povRight() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getPOV() == 90);}
    public Trigger povDownRight() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getPOV() == 135);}
    public Trigger povDown() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getPOV() == 180);}
    public Trigger povDownLeft() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getPOV() == 225);}
    public Trigger povLeft() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getPOV() == 270);}
    public Trigger povUpLeft() {return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getPOV() == 315);}


    public int getPOV() {
        return hid.getPOV();
    }

    public boolean isConnected() {
        return hid.isConnected();
    }
}
