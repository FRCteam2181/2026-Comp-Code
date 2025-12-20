package frc.robot.Constants.Configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ElevatorConstants;

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