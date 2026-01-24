package frc.robot.systems;

import java.util.Optional;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.newConstants.TurretConstants;

import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

//import frc.robot.subsystems.CoralPlacer;
//import frc.robot.subsystems.CoralFunnel;
//import frc.robot.subsystems.ElevatorSubsystem;
//import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
//import frc.robot.subsystems.Climber;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.Unit;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.OperatorConstants;
// import frc.robot.Setpoints.AutoScoring.HumanPlayer.Left;
// import frc.robot.subsystems.Blinkin;
// import frc.robot.subsystems.ElevatorSubsystemPID;
// import frc.robot.subsystems.CoralFunnel;
// import frc.robot.subsystems.CoralPlacer;
// import frc.robot.subsystems.AlgaeClaw;
// import frc.robot.subsystems.AlgaeRotator;
// import frc.robot.subsystems.Climber;
// import frc.robot.systems.TargetingSystem;
// import frc.robot.systems.TargetingSystem.ReefBranch;
//import frc.robot.systems.TargetingSystem.ReefSide;
//import frc.robot.systems.field.FieldConstants.CoralStation;


public class TurretTargetingSystem
{

  Optional<Pose2d> aimTarget = Optional.empty();
  double        omegaRadiansPerSecond = 0;
  // private CoralPlacer          m_coralPlacer;
  // private Elevator             m_elevator;
  private SwerveSubsystem      m_drivebase;
  // private TargetingSystem      m_targetSystem;
  // private CoralFunnel          m_coralFunnel;
  // private Climber              m_climber;

  public TurretTargetingSystem(
      // CoralPlacer coralPlacer,
      // Elevator elevator,
      SwerveSubsystem drivebase
      // TargetingSystem targeting,
      // CoralFunnel coralFunnel,
      // Climber climber
      )
  {
    // m_coralPlacer = coralPlacer;
    // m_elevator = elevator;
    m_drivebase = drivebase;

    // m_targetSystem = targeting;
    // m_coralFunnel = coralFunnel;
    // m_climber = climber;

  }


  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param turretPose the current pose2d of the turret on the field
   * @return Distance
   */
  public double getDistanceFromTarget()
  {
    Optional<Pose2d> turretTarget = aimTarget;
    return turretTarget.map(pose2d -> getDistanceToPose(pose2d)).orElse(-1.0);
  }


   /**
     * Returns the distance between two poses
     *
     * @param turretPose Pose of the robot
     * @param targetPose Pose of the target
     * @return distance to the pose
     */
    public double getDistanceToPose(Pose2d targetPose) {
        return getTurretPose().getTranslation().getDistance(targetPose.getTranslation());
    }

    /**
   * Gets the pose of the robot on the field
   * 
   * @return pose of the robot
   */
  public Pose2d getTurretPose() {
    return m_drivebase.getPose().transformBy(TurretConstants.ROBOT_TO_TURRET);
  }


  /**
   * Fetch the latest turret heading, should be trusted over {@link SwerveDrive#getYaw()}.
   *
   * @return {@link Rotation2d} of the robot heading.
   */
  public Rotation2d getTurretHeading()//this will need to pull the turrets current heading
  {
    return getTurretPose().getRotation();
  }



  /**
   * Calculate the angular velocity given the current and target heading angle in radians.
   *
   * @param currentHeadingAngleRadians The current heading of the robot in radians.
   * @param targetHeadingAngleRadians  The target heading of the robot in radians.
   * @return Angular velocity in radians per second.
   */
  public double headingCalculate()
  {
    Pose2d turretPose = getTurretPose();
    Rotation2d    currentTurretDirection = getTurretHeading();
    Translation2d relativeTrl    = aimTarget.get().relativeTo(turretPose).getTranslation();
    Rotation2d    target         = new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(currentTurretDirection);
    //omegaRadiansPerSecond = thetaController.calculate(currentTurretDirection.getRadians(), target.getRadians()) * config.maxAngularVelocity;
    
  
    return omegaRadiansPerSecond;
  }


}
