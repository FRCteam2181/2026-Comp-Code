package frc.robot.Utils.ControllerUtils.ElevatorBoard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IElevatorBoard {

    Trigger elevatorBoardButtonA ();

    Trigger elevatorBoardButtonB ();
    
    Trigger elevatorBoardButtonC ();
        
    Trigger elevatorBoardButtonD ();
        
    Trigger elevatorBoardButtonL1 ();
        
    Trigger elevatorBoardButtonR1 ();
        
    Trigger elevatorBoardButtonL2 ();

    Trigger elevatorBoardButtonR2 ();
        
    Trigger elevatorBoardButtonSelect ();
        
    Trigger elevatorBoardButtonStart ();
        
    Trigger elevatorBoardButtonL3 ();
        
    Trigger elevatorBoardButtonR3 ();
        
    Trigger elevatorBoardButtonUp ();
        
    Trigger elevatorBoardButtonDown ();
        
    Trigger elevatorBoardButtonRight ();
        
    Trigger elevatorBoardButtonLeft (); 
        
    Trigger elevatorBoardJoystickAsButtonPosX ();
            
    Trigger elevatorBoardJoystickAsButtonNegX ();

    Trigger elevatorBoardJoystickAsButtonPosY ();

    Trigger elevatorBoardJoystickAsButtonNegY ();

    }
