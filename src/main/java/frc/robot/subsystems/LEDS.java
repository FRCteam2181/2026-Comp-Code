// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.lumynlabs.devices.ConnectorXAnimate;

import java.time.ZoneId;

import com.lumynlabs.connection.usb.USBPort;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.util.Color;

public class LEDS extends SubsystemBase {

    private ConnectorXAnimate cXAnimate = new ConnectorXAnimate();

    boolean animateConnected = cXAnimate.Connect(USBPort.kUSB1);

    boolean isAllianceBlue = true;

    public LEDS() {

        checkAllianceColor();
        
    }

    public void checkAllianceColor() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            
            if (alliance.get() == DriverStation.Alliance.Blue) {
                isAllianceBlue = true;
            }
            if (alliance.get() == DriverStation.Alliance.Red) {
                isAllianceBlue = false;
            }
        }
    }

    public void setAllianceColor(String zoneID) {
        if (isAllianceBlue == true) {
            cXAnimate.leds.SetColor(zoneID, Color.kBlue);
        } else {
            cXAnimate.leds.SetColor(zoneID, Color.kRed);
        }
    }

    public void setGrowingBreath(String zoneID) {
        if (isAllianceBlue == true) {
            cXAnimate.leds.SetAnimationSequence(zoneID, "Blue-Growing-Breath");
        } else {
            cXAnimate.leds.SetAnimationSequence(zoneID, "Red-Growing-Breath");
        }
    }

    public void setGrowingBreath(String zoneID, Color color) {
        if (color == Color.kBlue) {
            cXAnimate.leds.SetAnimationSequence(zoneID, "Blue-Growing-Breath");
        } else if (color == Color.kRed) {
            cXAnimate.leds.SetAnimationSequence(zoneID, "Red-Growing-Breath");
        }
    }

    

    @Override
    public void periodic() {
        checkAllianceColor();
    }


}



