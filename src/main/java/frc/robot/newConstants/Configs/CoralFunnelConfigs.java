package frc.robot.newConstants.Configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.newConstants.CoralFunnelConstants;

import com.revrobotics.spark.config.SparkFlexConfig;

public class CoralFunnelConfigs {
    
    public static final SparkFlexConfig coralIntakeConfig = new SparkFlexConfig();

    static {
      //Configure the base settings for the Coral Intake Motor
      coralIntakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CoralFunnelConstants.k_CoralFunnelVoltageLimit);
    }
  }
