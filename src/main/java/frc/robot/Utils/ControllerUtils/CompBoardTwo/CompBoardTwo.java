package frc.robot.Utils.ControllerUtils.CompBoardTwo;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class CompBoardTwo implements ICompBoardTwo {

    private final Joystick CompBoardTwo;

    private static CompBoardTwo INSTANCE;

    public static CompBoardTwo getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CompBoardTwo();
        }
        return INSTANCE;
    }

    private CompBoardTwo() {
        CompBoardTwo = new Joystick(4);
    }

    @Override
    public Trigger CompBoardTwoButtonA () {
        return new JoystickButton(CompBoardTwo, 1);
    }


    @Override
    public Trigger CompBoardTwoButtonB () {
        return new JoystickButton(CompBoardTwo, 2);
    }

    @Override
    public Trigger CompBoardTwoButtonC () {
        return new JoystickButton(CompBoardTwo, 3);
    }

    @Override
    public Trigger CompBoardTwoButtonD () {
        return new JoystickButton(CompBoardTwo, 4);
    }

    @Override
    public Trigger CompBoardTwoButtonL1 () {
        return new JoystickButton(CompBoardTwo, 5);
    }

    @Override
    public Trigger CompBoardTwoButtonR1 () {
        return new JoystickButton(CompBoardTwo, 6);
    }

    @Override
    public Trigger CompBoardTwoButtonL2 () {
        return new JoystickButton(CompBoardTwo, 7);
    }

    @Override
    public Trigger CompBoardTwoButtonR2 () {
        return new JoystickButton(CompBoardTwo, 8);
    }

    @Override
    public Trigger CompBoardTwoButtonSelect () {
        return new JoystickButton(CompBoardTwo, 9);
    }

    @Override
    public Trigger CompBoardTwoButtonStart () {
        return new JoystickButton(CompBoardTwo, 10);
    }

    @Override
    public Trigger CompBoardTwoButtonL3 () {
        return new JoystickButton(CompBoardTwo, 11);
    }

    @Override
    public Trigger CompBoardTwoButtonR3 () {
        return new JoystickButton(CompBoardTwo, 12);
    }

    @Override
    public Trigger CompBoardTwoButtonUp () {
        return new JoystickButton(CompBoardTwo, 13);
    }

    @Override
    public Trigger CompBoardTwoButtonDown () {
        return new JoystickButton(CompBoardTwo, 14);
    }

    @Override
    public Trigger CompBoardTwoButtonRight () {
        return new JoystickButton(CompBoardTwo, 15);
    }

    @Override
    public Trigger CompBoardTwoButtonLeft () {
        return new JoystickButton(CompBoardTwo, 16);
    }

    @Override
    public Trigger CompBoardTwoJoystickAsButtonPosX() {
        return new Trigger(() -> CompBoardTwo.getX() > 0.5);
    }

    @Override
    public Trigger CompBoardTwoJoystickAsButtonNegX() {
        return new Trigger(() -> CompBoardTwo.getX() < -0.5);
    }

    @Override    
    public Trigger CompBoardTwoJoystickAsButtonPosY() {
        return new Trigger(() -> CompBoardTwo.getY() > 0.5);
    }

    @Override
    public Trigger CompBoardTwoJoystickAsButtonNegY() {
        return new Trigger(() -> CompBoardTwo.getY() < -0.5);
    }

}
