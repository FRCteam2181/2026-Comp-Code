package frc.robot.constants.Configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.CoralPlacerConstants;

import com.revrobotics.spark.config.SparkFlexConfig;

public class CoralPlacerConfigs {

    public static final SparkFlexConfig baseCoralPlacerConfig = new SparkFlexConfig();

    static {
      //Configure the base settings for the Coral Placer Motors
      baseCoralPlacerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CoralPlacerConstants.k_CoralPlacerVoltageLimit);
    }
  }
