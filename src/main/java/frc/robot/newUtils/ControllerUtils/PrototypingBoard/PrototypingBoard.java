package frc.robot.newUtils.ControllerUtils.PrototypingBoard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class PrototypingBoard implements IPrototypingBoard {

    private final Joystick prototypingBoard;

    private static PrototypingBoard INSTANCE;

    public static PrototypingBoard getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new PrototypingBoard();
        }
        return INSTANCE;
    }

    private PrototypingBoard() {
        prototypingBoard = new Joystick(4);
    }

    @Override
    public Trigger prototypingBoardButtonA () {
        return new JoystickButton(prototypingBoard, 1);
    }


    @Override
    public Trigger prototypingBoardButtonB () {
        return new JoystickButton(prototypingBoard, 2);
    }

    @Override
    public Trigger prototypingBoardButtonC () {
        return new JoystickButton(prototypingBoard, 3);
    }

    @Override
    public Trigger prototypingBoardButtonD () {
        return new JoystickButton(prototypingBoard, 4);
    }

    @Override
    public Trigger prototypingBoardButtonL1 () {
        return new JoystickButton(prototypingBoard, 5);
    }

    @Override
    public Trigger prototypingBoardButtonR1 () {
        return new JoystickButton(prototypingBoard, 6);
    }

    @Override
    public Trigger prototypingBoardButtonL2 () {
        return new JoystickButton(prototypingBoard, 7);
    }

    @Override
    public Trigger prototypingBoardButtonR2 () {
        return new JoystickButton(prototypingBoard, 8);
    }

    @Override
    public Trigger prototypingBoardButtonSelect () {
        return new JoystickButton(prototypingBoard, 9);
    }

    @Override
    public Trigger prototypingBoardButtonStart () {
        return new JoystickButton(prototypingBoard, 10);
    }

    @Override
    public Trigger prototypingBoardButtonL3 () {
        return new JoystickButton(prototypingBoard, 11);
    }

    @Override
    public Trigger prototypingBoardButtonR3 () {
        return new JoystickButton(prototypingBoard, 12);
    }

    @Override
    public Trigger prototypingBoardButtonUp () {
        return new JoystickButton(prototypingBoard, 13);
    }

    @Override
    public Trigger prototypingBoardButtonDown () {
        return new JoystickButton(prototypingBoard, 14);
    }

    @Override
    public Trigger prototypingBoardButtonRight () {
        return new JoystickButton(prototypingBoard, 15);
    }

    @Override
    public Trigger prototypingBoardButtonLeft () {
        return new JoystickButton(prototypingBoard, 16);
    }

    @Override
    public Trigger prototypingBoardJoystickAsButtonPosX() {
        return new Trigger(() -> prototypingBoard.getX() > 0.5);
    }

    @Override
    public Trigger prototypingBoardJoystickAsButtonNegX() {
        return new Trigger(() -> prototypingBoard.getX() < -0.5);
    }

    @Override    
    public Trigger prototypingBoardJoystickAsButtonPosY() {
        return new Trigger(() -> prototypingBoard.getY() > 0.5);
    }

    @Override
    public Trigger prototypingBoardJoystickAsButtonNegY() {
        return new Trigger(() -> prototypingBoard.getY() < -0.5);
    }

}
