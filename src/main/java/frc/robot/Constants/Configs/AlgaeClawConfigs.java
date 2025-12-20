package frc.robot.Constants.Configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.AlgaeClawConstants;

import com.revrobotics.spark.config.SparkMaxConfig;


public class AlgaeClawConfigs {

    public static final SparkMaxConfig baseAlgaeClawConfig = new SparkMaxConfig();
    
    static {
      //Configure base settings for the wheels on the Algae Claw
      baseAlgaeClawConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeClawConstants.k_AlgaeClawVoltageLimit);
    }  
  }