package frc.robot.systems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;

public class GameData extends SubsystemBase {

  // private CoralPlacer          m_coralPlacer;
  // private Elevator             m_elevator;
  private SwerveSubsystem m_drivebase;

  // private TargetingSystem      m_targetSystem;
  // private CoralFunnel          m_coralFunnel;
  // private Climber              m_climber;

  public GameData(
      // CoralPlacer coralPlacer,
      // Elevator elevator,
      SwerveSubsystem drivebase
      // TargetingSystem targeting,
      // CoralFunnel coralFunnel,
      // Climber climber
      ) {
    // m_coralPlacer = coralPlacer;
    // m_elevator = elevator;
    m_drivebase = drivebase;

    // m_targetSystem = targeting;
    // m_coralFunnel = coralFunnel;
    // m_climber = climber;

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("can_shoot", canShoot());
    SmartDashboard.putNumber("Match_Time", DriverStation.getMatchTime());
  }

  // public Trigger inScoringZone() {
  //   return new Trigger(
  //       () ->
  //           new Rectangle2d(
  //                   new Translation2d(0, 0),
  //                   new Translation2d(
  //                       FieldConstants.LinesVertical.starting, FieldConstants.fieldWidth))
  //               .contains(m_drivebase.getPose().getTranslation()));
  // }

  public boolean canShoot() {
    // String gameData;
    boolean canShootBool = true;
    if (DriverStation.getAlliance().isEmpty()) {
      System.out.println("In first");
      return true;
    }
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();
    boolean is_blue = (alliance == DriverStation.Alliance.Blue);
    String gameData = DriverStation.getGameSpecificMessage();
    double matchTime = DriverStation.getMatchTime();
    if (gameData.length() > 0) {
      // System.out.print("HellpoGabriella");
      switch (gameData.charAt(0)) {
        case 'B':
          // Blue case code
          if (is_blue && (matchTime <= 140 && matchTime >= 130)) {
            canShootBool = true;
          } else if (!is_blue && (matchTime <= 130 && matchTime >= 105)) {
            canShootBool = true;
          } else if (is_blue && (matchTime <= 105 && matchTime >= 55)) {
            canShootBool = true;
          } else if (!is_blue && (matchTime <= 55 && matchTime >= 30)) {
            canShootBool = true;
          }
          break;
        case 'R':
          if (!is_blue && (matchTime <= 140 && matchTime >= 130)) {
            canShootBool = false;
          } else if (is_blue && (matchTime <= 130 && matchTime >= 105)) {
            canShootBool = false;
          } else if (!is_blue && (matchTime <= 105 && matchTime >= 55)) {
            canShootBool = false;
          } else if (is_blue && (matchTime <= 55 && matchTime >= 30)) {
            canShootBool = false;
          }
          break;
          // Red case code

        default:
          // This is corrupt data
          canShootBool = true;
          break;
      }
    } else {
      // Code for no data received yet
      canShootBool = true;
      // System.out.print("IHATEMYLFIE");
    }
    return canShootBool;
  }
}
