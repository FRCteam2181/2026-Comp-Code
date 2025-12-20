package frc.robot.Constants.Configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.config.SparkMaxConfig;


public class ClimberConfigs {
    
    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {
      //Configure the base settings for the Climber motor
      climberConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.k_climberVoltageLimit);
    }   
  }
