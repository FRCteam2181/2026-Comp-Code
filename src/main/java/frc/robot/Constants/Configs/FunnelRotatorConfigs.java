package frc.robot.Constants.Configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.CoralFunnelConstants;

public class FunnelRotatorConfigs {
    
    public static final SparkFlexConfig funnelRotatorConfig = new SparkFlexConfig();

    static {
      //Configure the base settings for the coral funnel rotator motor
      funnelRotatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CoralFunnelConstants.k_CoralFunnelVoltageLimit);
    }
  }
