package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;

import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;

public class Vision extends SubsystemBase{
    /**
     * Ambiguity defined as a value between (0,1). Used in {@link Vision#filterPose}.
     */
    private final       double              maximumAmbiguity                = 0.25;
    /**
     * Photon Vision Simulation
     */
    public              VisionSystemSim     visionSim;
    /**
     * Count of times that the odom thinks we're more than 10meters away from the april tag.
     */
    private             double              longDistangePoseEstimationCount = 0;
    /**
     * Current pose from the pose estimator using wheel odometry.
     */
    private             Supplier<Pose2d>    currentPose;
    /**
     * Field from {@link swervelib.SwerveDrive#field}
     */
    private             Field2d             field2d;

    public static PhotonCamera camera;

    List<PhotonTrackedTarget> targets;

    /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, PhotonCamera camera)
  {
    this.currentPose = currentPose;
    this.camera = camera;
    //this.field2d = field;

    if (Robot.isSimulation())
    {
      visionSim = new VisionSystemSim("Vision");
      //visionSim.addAprilTags(fieldLayout);

      /*for (Cameras c : Cameras.values())
      {
        c.addToVisionSim(visionSim);
      }*/

      //openSimCameraViews();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(camera.getLatestResult().hasTargets()){
      SmartDashboard.putNumber("PhotonCamera distance from target", PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.ROBOT_TO_CAMERA.getZ(), 0.13208, 0, 0));
    }
    SmartDashboard.putNumber("PhotonVisionYaw", GetBestTargetYaw());
    SmartDashboard.putNumber("DetectedObjectRelativeX", camera.getLatestResult().getBestTarget().getBestCameraToTarget().getX());
    SmartDashboard.putNumber("DetectedObjectRelativeY", camera.getLatestResult().getBestTarget().getBestCameraToTarget().getY());
    SmartDashboard.putNumber("DetectedObjectRelativeZ", camera.getLatestResult().getBestTarget().getBestCameraToTarget().getZ());

    targets = camera.getLatestResult().getTargets();

    SmartDashboard.putBoolean("hasTargets", camera.getLatestResult().hasTargets());

  }

  /**
   * Calculates a target pose relative to the Robot Pose on the field.
   *
   * @param robotPose   The Robot's Pose on the field.
   * @return The pose of the best detected object.
   */
  public static Pose3d getBestObjectPose(Pose3d robotPose)
  {
    //Optional<Pose3d> objectPose3d = fieldLayout.getTagPose(aprilTag);
    if (camera.getLatestResult().hasTargets())
    {
      return robotPose.transformBy(VisionConstants.ROBOT_TO_CAMERA).transformBy(camera.getLatestResult().getBestTarget().getBestCameraToTarget());
    } else
    {
      throw new RuntimeException("Cannot get Best Object");
    }

  }


  /**
   * Calculates the yaw difference between the robot and a detected object.
   *
   * @return The yaw difference between the robot and a detected object.
   */
  public static double GetBestTargetYaw(){
    if(camera.getLatestResult().hasTargets()){
      double yaw = camera.getLatestResult().getBestTarget().getYaw();
      return yaw;
    }else{
      return 0;
    }
    
  }
}
