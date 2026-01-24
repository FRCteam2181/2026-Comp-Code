package frc.robot.constants.Configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.AlgaeRotatorConstants;

import com.revrobotics.spark.config.SparkMaxConfig;


public class AlgaeRotatorConfigs {

    public static final SparkMaxConfig algaeRotatorConfig = new SparkMaxConfig();
    
    static {
      // Configure basic settings of the Algae Claw Rotation motor
      algaeRotatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeRotatorConstants.kAlgaeArmStallCurrentLimitAmps)
        .softLimit.reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(30)
        .forwardSoftLimitEnabled(true);
    }  
  }