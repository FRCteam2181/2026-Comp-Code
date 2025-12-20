package frc.robot.Utils.ControllerUtils.PositioningBoard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class PositioningBoard implements IPositioningBoard {

    private final Joystick positioningBoard;

    private static PositioningBoard INSTANCE;

    public static PositioningBoard getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new PositioningBoard();
        }
        return INSTANCE;
    }

    private PositioningBoard() {
        positioningBoard = new Joystick(4);
    }

    @Override
    public Trigger positioningBoardButtonA () {
        return new JoystickButton(positioningBoard, 1);
    }


    @Override
    public Trigger positioningBoardButtonB () {
        return new JoystickButton(positioningBoard, 2);
    }

    @Override
    public Trigger positioningBoardButtonC () {
        return new JoystickButton(positioningBoard, 3);
    }

    @Override
    public Trigger positioningBoardButtonD () {
        return new JoystickButton(positioningBoard, 4);
    }

    @Override
    public Trigger positioningBoardButtonL1 () {
        return new JoystickButton(positioningBoard, 5);
    }

    @Override
    public Trigger positioningBoardButtonR1 () {
        return new JoystickButton(positioningBoard, 6);
    }

    @Override
    public Trigger positioningBoardButtonL2 () {
        return new JoystickButton(positioningBoard, 7);
    }

    @Override
    public Trigger positioningBoardButtonR2 () {
        return new JoystickButton(positioningBoard, 8);
    }

    @Override
    public Trigger positioningBoardButtonSelect () {
        return new JoystickButton(positioningBoard, 9);
    }

    @Override
    public Trigger positioningBoardButtonStart () {
        return new JoystickButton(positioningBoard, 10);
    }

    @Override
    public Trigger positioningBoardButtonL3 () {
        return new JoystickButton(positioningBoard, 11);
    }

    @Override
    public Trigger positioningBoardButtonR3 () {
        return new JoystickButton(positioningBoard, 12);
    }

    @Override
    public Trigger positioningBoardButtonUp () {
        return new JoystickButton(positioningBoard, 13);
    }

    @Override
    public Trigger positioningBoardButtonDown () {
        return new JoystickButton(positioningBoard, 14);
    }

    @Override
    public Trigger positioningBoardButtonRight () {
        return new JoystickButton(positioningBoard, 15);
    }

    @Override
    public Trigger positioningBoardButtonLeft () {
        return new JoystickButton(positioningBoard, 16);
    }

    @Override
    public Trigger positioningBoardJoystickAsButtonPosX() {
        return new Trigger(() -> positioningBoard.getX() > 0.5);
    }

    @Override
    public Trigger positioningBoardJoystickAsButtonNegX() {
        return new Trigger(() -> positioningBoard.getX() < -0.5);
    }

    @Override    
    public Trigger positioningBoardJoystickAsButtonPosY() {
        return new Trigger(() -> positioningBoard.getY() > 0.5);
    }

    @Override
    public Trigger positioningBoardJoystickAsButtonNegY() {
        return new Trigger(() -> positioningBoard.getY() < -0.5);
    }

}
