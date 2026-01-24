package frc.robot.newConstants.Configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.newConstants.CoralFunnelConstants;

import com.revrobotics.spark.config.SparkFlexConfig;

public class FunnelRotatorConfigs {
    
    public static final SparkFlexConfig funnelRotatorConfig = new SparkFlexConfig();

    static {
      //Configure the base settings for the coral funnel rotator motor
      funnelRotatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CoralFunnelConstants.k_CoralFunnelVoltageLimit);
    }
  }
