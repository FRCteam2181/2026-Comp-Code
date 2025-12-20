package frc.robot.Constants.Configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.CoralPlacerConstants;

public class CoralPlacerConfigs {

    public static final SparkFlexConfig baseCoralPlacerConfig = new SparkFlexConfig();

    static {
      //Configure the base settings for the Coral Placer Motors
      baseCoralPlacerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CoralPlacerConstants.k_CoralPlacerVoltageLimit);
    }
  }
