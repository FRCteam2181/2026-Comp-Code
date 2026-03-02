package frc.robot.constants;

import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Configs {
  public static final class climberConfigs {

    public static final SoftLimitConfig climberSoftLimits =
        new SoftLimitConfig()
            .reverseSoftLimit(0)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(0)
            .forwardSoftLimitEnabled(true);

    public static final SparkBaseConfig climberLConfig =
        new SparkMaxConfig().idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false);
    // .apply(climberSoftLimits);

    public static final SparkBaseConfig climberRConfig =
        new SparkMaxConfig().idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false);

    public static final SparkBaseConfig shooterConfig =
        new SparkFlexConfig().idleMode(IdleMode.kCoast);
  }
}
