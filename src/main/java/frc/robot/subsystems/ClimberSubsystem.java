package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.climberConfigs;
import frc.robot.NewConstants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  SparkMax m_LeftClimber;
  SparkMax m_RightClimber;

  public ClimberSubsystem() {
    m_LeftClimber = new SparkMax(ClimberConstants.kLeftClimber_ID, MotorType.kBrushless);
    // m_climberR = new SparkMax(33, MotorType.kBrushless);

    m_LeftClimber.configure(
        climberConfigs.climberLConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    //  m_climberR.configure(climberConfigs.climberRConfig.follow(m_climberL, false),
    // ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public Command c_climb() {
    return this.startEnd(
        () -> {
          m_LeftClimber.set(ClimberConstants.kClimberSpeed);
        },
        () -> {
          m_LeftClimber.set(0);
        });
  }

  public Command c_climbReverse() {
    return this.startEnd(
        () -> {
          m_LeftClimber.set(-ClimberConstants.kClimberSpeed);
        },
        () -> {
          m_LeftClimber.set(0);
        });
  }
}
