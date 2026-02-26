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
import frc.robot.constants.VisionConstants;

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
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;

public class Vision extends SubsystemBase{

    public static final AprilTagFieldLayout fieldLayout                     = AprilTagFieldLayout.loadField(
      AprilTagFields.k2026RebuiltWelded);
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

    public static PhotonCamera camera, camera1, camera2;
    public PhotonCameraSim cameraSim, cameraSim1, cameraSim2;

    // List<PhotonTrackedTarget> targetsSim;
    ArrayList<PhotonPipelineResult> targets;


    /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, PhotonCamera camera, Field2d field)
  {
    this.currentPose = currentPose;
    this.camera = camera;
    /*PhotonCamera camera1 = new PhotonCamera("PhotonCamera1");
    PhotonCamera camera2 = new PhotonCamera("PhotonCamera2");*/
    this.field2d = field;

    if (Robot.isSimulation())
    {
      visionSim = new VisionSystemSim("Vision");
      //visionSim.addAprilTags(fieldLayout);
      //visionSim.addAprilTags(fieldLayout);

      TargetModel object = new TargetModel(0.5, 0.25);

      // The pose of where the target is on the field.
      // Its rotation determines where "forward" or the target x-axis points.
      // Let's say this target is flat against the far wall center, facing the blue driver stations.
      Pose3d targetPose = new Pose3d(8, 4, 0, new Rotation3d(0, 0, Math.PI));
      // The given target model at the given pose
      VisionTargetSim visionTarget = new VisionTargetSim(targetPose, object);

      // Add this vision target to the vision system simulation to make it visible
      visionSim.addVisionTargets(visionTarget);

      // Add Camera(s)
      // Properties
      SimCameraProperties cameraProp = new SimCameraProperties();

      cameraSim = new PhotonCameraSim(camera, cameraProp);
      /*cameraSim1 = new PhotonCameraSim(camera1, cameraProp);
      cameraSim2 = new PhotonCameraSim(camera2, cameraProp);*/

      // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        // Add this camera to the vision system simulation with the given robot-to-camera transform.
        visionSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAMERA);
        /*visionSim.addCamera(cameraSim1, new Transform3d(15,15,0,new Rotation3d(0,0,Units.degreesToRadians(45))));
        visionSim.addCamera(cameraSim2, new Transform3d(15,-15,0,new Rotation3d(0,0,Units.degreesToRadians(-45))));*/


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

    targets = new ArrayList<PhotonPipelineResult>(camera.getAllUnreadResults());

    // var result = targets.get(targets.size() - 1);


    if(!targets.isEmpty()){
      try{
        if(targets.get(targets.size() - 1).hasTargets()){
          // SmartDashboard.putNumber("PhotonCamera distance from target", PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.ROBOT_TO_CAMERA.getZ(), 0.13208, 0, 0));
          SmartDashboard.putNumber("PhotonCamera distance from target", getDistanceToTarget());
          SmartDashboard.putNumber("PhotonVisionYaw", getBestTargetYaw());
          // SmartDashboard.putNumber("DetectedObjectRelativeX", targets.get(targets.size() - 1).getBestTarget().getBestCameraToTarget().getX());
          // SmartDashboard.putNumber("DetectedObjectRelativeY", targets.get(targets.size() - 1).getBestTarget().getBestCameraToTarget().getY());
          // SmartDashboard.putNumber("DetectedObjectRelativeZ", targets.get(targets.size() - 1).getBestTarget().getBestCameraToTarget().getZ());
          SmartDashboard.putNumber("DetectedObjectRelativeX", getBestCameraToTarget().getX());
          SmartDashboard.putNumber("DetectedObjectRelativeY", getBestCameraToTarget().getY());
          SmartDashboard.putNumber("DetectedObjectRelativeZ", getBestCameraToTarget().getZ());
          SmartDashboard.putNumber("DetectedObjectPoseAmbiguity", targets.get(targets.size() - 1).getBestTarget().getPoseAmbiguity());
          SmartDashboard.putBoolean("hasTargets", targets.get(targets.size() - 1).hasTargets());
          // System.out.println("getBestTarget() = " + targets.get(targets.size() - 1).getBestTarget());
          // getDistanceToTarget();
          SmartDashboard.putNumber("Target X", getBestObjectPose().getX());
          SmartDashboard.putNumber("Target Y", getBestObjectPose().getY());
          SmartDashboard.putNumber("Target Z", getBestObjectPose().getZ());
        }
      } catch(IndexOutOfBoundsException a) {
        System.out.println("ts code tweaking");
      }
      
    }
    
    // visionSim.update(currentPose.get());

    SmartDashboard.putBoolean("isEmpty", targets.isEmpty());

  }




  /**
   * Calculates a target pose relative to the Robot Pose on the field in simulation.
   *
   * @return The pose of the best detected object.
   */
  public Pose3d getBestObjectPoseSim()
  {
    //Optional<Pose3d> objectPose3d = fieldLayout.getTagPose(aprilTag);
    
    if (!visionSim.getVisionTargets().isEmpty())
    {
      ArrayList<VisionTargetSim> visionTargetsArray = new ArrayList<>(visionSim.getVisionTargets());

      //visionTargetsArray.get(0).getPose();

      //PhotonPipelineResult camResult = cameraSim.process(0, new Pose3d(currentPose.get()).transformBy(VisionConstants.ROBOT_TO_CAMERA), visionTargetsArray);
      // publish this info to NT at estimated timestamp of receive
      //cameraSim.submitProcessedFrame(camResult, timestampNT);
      // display debug results
      //Transform3d trf = camResult.getBestTarget().getBestCameraToTarget();
      return visionTargetsArray.get(0).getPose();
      //System.out.println(new Pose3d(currentPose.get()).transformBy(VisionConstants.ROBOT_TO_CAMERA));
      // return new Pose3d(currentPose.get()).transformBy(VisionConstants.ROBOT_TO_CAMERA).transformBy(trf);
    } else
    {
      //throw new RuntimeException("Cannot get Best Object");
      return new Pose3d();
    }

  }

  public double getDistanceToTarget(){
    if(!targets.isEmpty()){
      try{
        if(targets.get(targets.size() - 1).hasTargets()){
          double pixelRatio = Math.sqrt(targets.get(targets.size() - 1).getBestTarget().getArea())/(Math.pow(5.9,2));
          double base = 24*(Math.sqrt(7.23)/(Math.pow(5.9,2)));
          double dist = base/pixelRatio;
          System.out.println("distance = " + dist + ", area = " + Math.sqrt(targets.get(targets.size() - 1).getBestTarget().getArea()));
          return dist;
        }
      } catch(IndexOutOfBoundsException a) {
        System.out.println("ts code tweaking #2: electric boogaloo");
      }
      
    }

    return 0;
  }

  public Pose3d getBestTargetPose() {
    // double distance = getDistanceToTarget()
    Pose3d position = new Pose3d(new Translation3d(currentPose.get().getTranslation()).plus(getBestCameraToTarget().rotateBy(new Rotation3d(currentPose.get().getRotation()))), new Rotation3d(currentPose.get().getRotation()));
    return position;
  }

  public Translation3d getBestCameraToTarget(){
    if(!targets.isEmpty()){
      try{
        if(targets.get(targets.size() - 1).hasTargets()){
          double dist = getDistanceToTarget();
          double yaw = targets.get(targets.size() - 1).getBestTarget().getYaw();
          double pitch = targets.get(targets.size() - 1).getBestTarget().getPitch();
          double x_dist = dist*Math.cos(Units.degreesToRadians(yaw));
          double y_dist = dist*Math.sin(Units.degreesToRadians(yaw));
          double z_dist = dist*Math.sin(Units.degreesToRadians(pitch));

          System.out.println(new Translation3d(x_dist,y_dist,z_dist));

          return new Translation3d(x_dist,y_dist,z_dist);
        }
      } catch(IndexOutOfBoundsException a) {
        System.out.println("ts code tweaking #3: electric boogalee");
      }
      
    }

    return new Translation3d();
  }

  /**
   * Calculates a target pose relative to the Robot Pose on the field.
   *
   * @param currentPose.get()   The Robot's Pose on the field.
   * @return The pose of the best detected object.
   */
  public Pose3d getBestObjectPose()
  {
    //Optional<Pose3d> objectPose3d = fieldLayout.getTagPose(aprilTag);
    if (!targets.isEmpty() && targets.get(targets.size() - 1).hasTargets())
    {
      return new Pose3d(currentPose.get()).transformBy(VisionConstants.ROBOT_TO_CAMERA).transformBy(targets.get(targets.size() - 1).getBestTarget().getBestCameraToTarget());
    } else
    {
      throw new RuntimeException("Cannot get Best Object");
    }

  }

  /**
   * Calculates the yaw difference between the robot and a detected object in simulation.
   *
   * @return The yaw difference between the robot and a detected object.
   */
  public double getBestTargetYawSim(){
    if(!visionSim.getVisionTargets().isEmpty()){
      ArrayList<VisionTargetSim> visionTargetsArray = new ArrayList<VisionTargetSim>(visionSim.getVisionTargets());
      //System.out.println(visionTargetsArray.get(0));

      var camResult = cameraSim.process(0, new Pose3d(currentPose.get()).transformBy(VisionConstants.ROBOT_TO_CAMERA), visionTargetsArray);
      //System.out.println(camResult.getTargets());
      //System.out.println(camResult.getBestTarget().getYaw());
      if(camResult.hasTargets()){
        return camResult.getBestTarget().getYaw();
      } else
        return 0;
      
    }else{
      return 0;
    }
    
  }
  


  /**
   * Calculates the yaw difference between the robot and a detected object.
   *
   * @return The yaw difference between the robot and a detected object.
   */
  public double getBestTargetYaw(){
    if(!targets.isEmpty() && targets.get(targets.size() - 1).hasTargets()){
      double yaw = targets.get(targets.size() - 1).getBestTarget().getYaw();
      return yaw;
    }else{
      return 0;
    }
    
  }
}
