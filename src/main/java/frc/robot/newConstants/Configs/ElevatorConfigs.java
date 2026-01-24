package frc.robot.newConstants.Configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.newConstants.ElevatorConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorConfigs {

    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig baseElevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      baseElevatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
        .closedLoopRampRate(ElevatorConstants.kElevatorRampRate);

      elevatorConfig
        .apply(baseElevatorConfig);

      elevatorFollowerConfig
        .apply(baseElevatorConfig);
    }       
  }