package frc.robot.Utils.ControllerUtils.ElevatorBoard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ElevatorBoard implements IElevatorBoard {

    private final Joystick elevatorBoard;

    private static ElevatorBoard INSTANCE;

    public static ElevatorBoard getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ElevatorBoard();
        }
        return INSTANCE;
    }

    private ElevatorBoard() {
        elevatorBoard = new Joystick(4);
    }

    @Override
    public Trigger elevatorBoardButtonA () {
        return new JoystickButton(elevatorBoard, 1);
    }


    @Override
    public Trigger elevatorBoardButtonB () {
        return new JoystickButton(elevatorBoard, 2);
    }

    @Override
    public Trigger elevatorBoardButtonC () {
        return new JoystickButton(elevatorBoard, 3);
    }

    @Override
    public Trigger elevatorBoardButtonD () {
        return new JoystickButton(elevatorBoard, 4);
    }

    @Override
    public Trigger elevatorBoardButtonL1 () {
        return new JoystickButton(elevatorBoard, 5);
    }

    @Override
    public Trigger elevatorBoardButtonR1 () {
        return new JoystickButton(elevatorBoard, 6);
    }

    @Override
    public Trigger elevatorBoardButtonL2 () {
        return new JoystickButton(elevatorBoard, 7);
    }

    @Override
    public Trigger elevatorBoardButtonR2 () {
        return new JoystickButton(elevatorBoard, 8);
    }

    @Override
    public Trigger elevatorBoardButtonSelect () {
        return new JoystickButton(elevatorBoard, 9);
    }

    @Override
    public Trigger elevatorBoardButtonStart () {
        return new JoystickButton(elevatorBoard, 10);
    }

    @Override
    public Trigger elevatorBoardButtonL3 () {
        return new JoystickButton(elevatorBoard, 11);
    }

    @Override
    public Trigger elevatorBoardButtonR3 () {
        return new JoystickButton(elevatorBoard, 12);
    }

    @Override
    public Trigger elevatorBoardButtonUp () {
        return new JoystickButton(elevatorBoard, 13);
    }

    @Override
    public Trigger elevatorBoardButtonDown () {
        return new JoystickButton(elevatorBoard, 14);
    }

    @Override
    public Trigger elevatorBoardButtonRight () {
        return new JoystickButton(elevatorBoard, 15);
    }

    @Override
    public Trigger elevatorBoardButtonLeft () {
        return new JoystickButton(elevatorBoard, 16);
    }

    @Override
    public Trigger elevatorBoardJoystickAsButtonPosX() {
        return new Trigger(() -> elevatorBoard.getX() > 0.5);
    }

    @Override
    public Trigger elevatorBoardJoystickAsButtonNegX() {
        return new Trigger(() -> elevatorBoard.getX() < -0.5);
    }

    @Override    
    public Trigger elevatorBoardJoystickAsButtonPosY() {
        return new Trigger(() -> elevatorBoard.getY() > 0.5);
    }

    @Override
    public Trigger elevatorBoardJoystickAsButtonNegY() {
        return new Trigger(() -> elevatorBoard.getY() < -0.5);
    }

}
