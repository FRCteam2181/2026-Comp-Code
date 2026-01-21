package frc.robot.subsystems;

//import java.util.Optional;

//import org.ejml.equation.Variable;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class GameData extends SubsystemBase {
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("can_shoot", canShoot());
    }

    public boolean canShoot() {
        //String gameData;
        boolean canShootBool = true;
        DriverStation.Alliance alliance = DriverStation.getAlliance().get();
        boolean is_blue = (alliance == DriverStation.Alliance.Blue);
        String gameData = DriverStation.getGameSpecificMessage();
        double matchTime = DriverStation.getMatchTime();
        if(gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B' :
                    //Blue case code
                    if (is_blue && (matchTime >= 140 && matchTime <= 130)) {
                        canShootBool = false;
                    }
                    else if (!is_blue && (matchTime >= 130 && matchTime <= 105)) {
                        canShootBool = false;
                    }
                    else if (is_blue && (matchTime >= 105 && matchTime <= 55)) {
                        canShootBool = false;
                    }
                    else if (!is_blue && (matchTime >= 55 && matchTime <= 30)) {
                        canShootBool = false;
                    }
                    break;
                case 'R' :
                    if (!is_blue && (matchTime >= 140 && matchTime <= 130)) {
                        canShootBool = false;
                    }
                    else if (is_blue && (matchTime >= 130 && matchTime <= 105)) {
                        canShootBool = false;
                    }
                    else if (!is_blue && (matchTime >= 105 && matchTime <= 55)) {
                        canShootBool = false;
                    }
                    else if (is_blue && (matchTime >= 55 && matchTime <= 30)) {
                        canShootBool = false;
                    }
                    //Red case code
                default :
                    //This is corrupt data
                    canShootBool = true;
                    break;
            }
        } else {
            //Code for no data received yet
            canShootBool = true;
        }
        return canShootBool;
    }
}

