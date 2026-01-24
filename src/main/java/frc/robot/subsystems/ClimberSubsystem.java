package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Configs.ClimberConfigs.climberConfigs;

public class ClimberSubsystem extends SubsystemBase{
    SparkMax m_climberL;
    SparkMax m_climberR;
    
    public ClimberSubsystem(){
        m_climberL = new SparkMax(15, MotorType.kBrushless);
       // m_climberR = new SparkMax(33, MotorType.kBrushless);

        m_climberL.configure(climberConfigs.climberLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      //  m_climberR.configure(climberConfigs.climberRConfig.follow(m_climberL, false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }

    public Command c_climb(){
        return this.startEnd(() -> {
            m_climberL.set(.7);


        }, 
        
        () -> {
        m_climberL.set(0);

            
        });
    }
    public Command c_climbReverse(){
            return this.startEnd(() -> {
                m_climberL.set(-.7);
    
    
            }, 
            
            () -> {
            m_climberL.set(0);
    
                
            });
    }
}