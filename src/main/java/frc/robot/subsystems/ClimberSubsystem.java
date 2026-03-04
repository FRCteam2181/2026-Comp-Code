package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Configs.climberConfigs;
import java.util.function.BooleanSupplier;

public class ClimberSubsystem extends SubsystemBase {

  SparkMax m_LeftClimber;
  RelativeEncoder climberRelative;

  public ClimberSubsystem() {
    m_LeftClimber = new SparkMax(ClimberConstants.kLeftClimber_ID, MotorType.kBrushless);
    climberRelative = m_LeftClimber.getEncoder();

    // m_climberR = new SparkMax(33, MotorType.kBrushless);

    m_LeftClimber.configure(
        climberConfigs.climberLConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    //  m_climberR.configure(climberConfigs.climberRConfig.follow(m_climberL, false),
    // ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public BooleanSupplier hitForwrdLimit() {

    return (() -> m_LeftClimber.getForwardSoftLimit().isReached());
  }

  public BooleanSupplier hitReverseLimit() {

    return (() -> m_LeftClimber.getReverseSoftLimit().isReached());
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Climber Position", climberRelative.getPosition());
  }
}
