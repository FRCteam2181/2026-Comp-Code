package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfig.IdleMode;

public class Configs {
    public static final class climberConfigs{

        public static final SparkMaxConfig climberLConfig = new SparkMaxConfig().idleMode(IdleMode.kCoast);

    }
    
}
