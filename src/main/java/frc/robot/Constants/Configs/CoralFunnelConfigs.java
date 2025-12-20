package frc.robot.Constants.Configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.CoralFunnelConstants;

public class CoralFunnelConfigs {
    
    public static final SparkFlexConfig coralIntakeConfig = new SparkFlexConfig();

    static {
      //Configure the base settings for the Coral Intake Motor
      coralIntakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CoralFunnelConstants.k_CoralFunnelVoltageLimit);
    }
  }
