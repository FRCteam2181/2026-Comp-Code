package frc.robot.newConstants.Configs;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ClimberConfigs {
    public static final class climberConfigs{

        public static final SparkBaseConfig climberLConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake)    .smartCurrentLimit(40)
        .inverted(false);
        public static final SparkBaseConfig climberRConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake)    .smartCurrentLimit(40)
        .inverted(false);

    }
    
}
