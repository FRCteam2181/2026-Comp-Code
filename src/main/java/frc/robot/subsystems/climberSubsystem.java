package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climberSubsystem extends SubsystemBase{
    SparkMax m_climberL;
    SparkMax m_climberR;
    
    public climberSubsystem(){
        m_climberL = new SparkMax(0, MotorType.kBrushless);
        m_climberR = new SparkMax(1, MotorType.kBrushless);

        m_climberL.configure(null, null, null)

    }
}
