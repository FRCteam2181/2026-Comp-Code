package frc.robot.Utils.ControllerUtils.PositioningBoard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IPositioningBoard {

    Trigger positioningBoardButtonA ();

    Trigger positioningBoardButtonB ();
    
    Trigger positioningBoardButtonC ();
        
    Trigger positioningBoardButtonD ();
        
    Trigger positioningBoardButtonL1 ();
        
    Trigger positioningBoardButtonR1 ();
        
    Trigger positioningBoardButtonL2 ();

    Trigger positioningBoardButtonR2 ();
        
    Trigger positioningBoardButtonSelect ();
        
    Trigger positioningBoardButtonStart ();
        
    Trigger positioningBoardButtonL3 ();
        
    Trigger positioningBoardButtonR3 ();
        
    Trigger positioningBoardButtonUp ();
        
    Trigger positioningBoardButtonDown ();
        
    Trigger positioningBoardButtonRight ();
        
    Trigger positioningBoardButtonLeft (); 
        
    Trigger positioningBoardJoystickAsButtonPosX ();
            
    Trigger positioningBoardJoystickAsButtonNegX ();

    Trigger positioningBoardJoystickAsButtonPosY ();

    Trigger positioningBoardJoystickAsButtonNegY ();

    }
