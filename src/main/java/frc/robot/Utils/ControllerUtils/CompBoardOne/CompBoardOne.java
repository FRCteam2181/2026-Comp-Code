package frc.robot.Utils.ControllerUtils.CompBoardOne;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class CompBoardOne implements ICompBoardOne {

    private final Joystick CompBoardOne;

    private static CompBoardOne INSTANCE;

    public static CompBoardOne getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CompBoardOne();
        }
        return INSTANCE;
    }

    private CompBoardOne() {
        CompBoardOne = new Joystick(4);
    }

    @Override
    public Trigger CompBoardOneButtonA () {
        return new JoystickButton(CompBoardOne, 1);
    }


    @Override
    public Trigger CompBoardOneButtonB () {
        return new JoystickButton(CompBoardOne, 2);
    }

    @Override
    public Trigger CompBoardOneButtonC () {
        return new JoystickButton(CompBoardOne, 3);
    }

    @Override
    public Trigger CompBoardOneButtonD () {
        return new JoystickButton(CompBoardOne, 4);
    }

    @Override
    public Trigger CompBoardOneButtonL1 () {
        return new JoystickButton(CompBoardOne, 5);
    }

    @Override
    public Trigger CompBoardOneButtonR1 () {
        return new JoystickButton(CompBoardOne, 6);
    }

    @Override
    public Trigger CompBoardOneButtonL2 () {
        return new JoystickButton(CompBoardOne, 7);
    }

    @Override
    public Trigger CompBoardOneButtonR2 () {
        return new JoystickButton(CompBoardOne, 8);
    }

    @Override
    public Trigger CompBoardOneButtonSelect () {
        return new JoystickButton(CompBoardOne, 9);
    }

    @Override
    public Trigger CompBoardOneButtonStart () {
        return new JoystickButton(CompBoardOne, 10);
    }

    @Override
    public Trigger CompBoardOneButtonL3 () {
        return new JoystickButton(CompBoardOne, 11);
    }

    @Override
    public Trigger CompBoardOneButtonR3 () {
        return new JoystickButton(CompBoardOne, 12);
    }

    @Override
    public Trigger CompBoardOneButtonUp () {
        return new JoystickButton(CompBoardOne, 13);
    }

    @Override
    public Trigger CompBoardOneButtonDown () {
        return new JoystickButton(CompBoardOne, 14);
    }

    @Override
    public Trigger CompBoardOneButtonRight () {
        return new JoystickButton(CompBoardOne, 15);
    }

    @Override
    public Trigger CompBoardOneButtonLeft () {
        return new JoystickButton(CompBoardOne, 16);
    }

    @Override
    public Trigger CompBoardOneJoystickAsButtonPosX() {
        return new Trigger(() -> CompBoardOne.getX() > 0.5);
    }

    @Override
    public Trigger CompBoardOneJoystickAsButtonNegX() {
        return new Trigger(() -> CompBoardOne.getX() < -0.5);
    }

    @Override    
    public Trigger CompBoardOneJoystickAsButtonPosY() {
        return new Trigger(() -> CompBoardOne.getY() > 0.5);
    }

    @Override
    public Trigger CompBoardOneJoystickAsButtonNegY() {
        return new Trigger(() -> CompBoardOne.getY() < -0.5);
    }

}
