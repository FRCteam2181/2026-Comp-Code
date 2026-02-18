package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.systems.ScoringSystem;
import java.util.Map;
import java.util.function.Supplier;

public class ShootOnTheMoveCommand extends Command {
  private final SwerveSubsystem drivetrain;
  private final ScoringSystem superstructure;

  private Supplier<Translation3d> aimPointSupplier; // The point to aim at
  private AngularVelocity latestShootSpeed;
  private Angle latestHoodAngle;
  private Angle latestTurretAngle;

  public ShootOnTheMoveCommand(
      SwerveSubsystem drivetrain,
      ScoringSystem superstructure,
      Supplier<Translation3d> aimPointSupplier) {
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.aimPointSupplier = aimPointSupplier;

    // We use the drivetrain to determine linear velocity, but don't require it for
    // control. We
    // will be using the superstructure to control the shooting mechanism so it's a
    // requirement.
    // addRequirements(superstructure);

    // TODO: figure out if the above is actually required. Right now, when you start
    // some other command, the auto aim can't start back up again
  }

  @Override
  public void initialize() {
    super.initialize();

    latestHoodAngle = superstructure.getHoodAngle();
    latestTurretAngle = superstructure.getTurretAngle();
    latestShootSpeed = superstructure.getShooterSpeed();

    // TODO: when this current command ends, we should probably cancel the dynamic
    // aim command
    superstructure
        .aimDynamicCommand(
            () -> {
              return this.latestShootSpeed;
            },
            () -> {
              return this.latestTurretAngle;
            },
            () -> {
              return this.latestHoodAngle;
            })
        .schedule();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    // Calculate trajectory to aimPoint
    var target = aimPointSupplier.get();

    var shooterLocation =
        drivetrain
            .getPose3d()
            .getTranslation()
            .plus(superstructure.getShooterPose().getTranslation());

    // Ignore this parameter for now, the range tables will account for it :/
    // var deltaH = target.getMeasureZ().minus(shooterLocation.getMeasureZ());
    var shooterOnGround = new Translation2d(shooterLocation.getX(), shooterLocation.getY());
    var targetOnGround = new Translation2d(target.getX(), target.getY());

    var distanceToTarget = Meters.of(shooterOnGround.getDistance(targetOnGround));

    // Get time of flight. We could try to do this analytically but for now it's
    // easier and more realistic
    // to use a simple linear approximation based on empirical data.
    double timeOfFlight = getFlightTime(distanceToTarget);

    // Calculate corrective vector based on our current velocity multiplied by time
    // of flight.
    // If we're stationary, this should be zero. If we're backing up, this will be
    // "ahead" of the target, etc.
    var updatedPosition = drivetrain.getFieldVelocity().times(timeOfFlight);
    var correctiveVector =
        new Translation2d(updatedPosition.vxMetersPerSecond, updatedPosition.vyMetersPerSecond)
            .unaryMinus();
    var correctiveVector3d = new Translation3d(correctiveVector.getX(), correctiveVector.getY(), 0);

    // Logger.recordOutput("FieldSimulation/AimTargetCorrected",
    //     new Pose3d(target.plus(correctiveVector3d), Rotation3d.kZero));

    var correctedTarget = targetOnGround.plus(correctiveVector);

    var vectorToTarget = drivetrain.getPose().getTranslation().minus(correctedTarget);

    var correctedDistance = Meters.of(vectorToTarget.getNorm());
    var calculatedHeading =
        vectorToTarget.getAngle().rotateBy(drivetrain.getHeading()).getMeasure(); // .negate();
    // .minus(Degrees.of(180)); // .unaryminus

    // Logger.recordOutput("ShootOnTheMove/RobotHeading", drivetrain.getHeading());
    // Logger.recordOutput("ShootOnTheMove/CalculatedHeading", calculatedHeading);
    // Logger.recordOutput("ShootOnTheMove/distanceToTarget", distanceToTarget);

    latestTurretAngle = calculatedHeading;
    latestShootSpeed = calculateRequiredShooterSpeed(correctedDistance);

    // TODO: add this back if/when we have a real hood, for now, just set it to the
    // current angle
    // latestHoodAngle = calculateRequiredHoodAngle(correctedDistance);
    latestHoodAngle = superstructure.getHoodAngle();

    superstructure.setShooterSetpoints(latestShootSpeed, latestTurretAngle, latestHoodAngle);

    // SmartDashboard.putNumber("shoot speed", latestShootSpeed);
    // SmartDashboard.putNumber("target angle", latestTurretAngle);
    // SmartDashboard.putBoolean("Encoder A Raw", rotorSeededFromAbs);

    // System.out.println("Shooting at distance: " + correctedDistance + " requires
    // speed: " + latestShootSpeed
    // + ", hood angle: " + latestHoodAngle + ", turret angle: " +
    // latestTurretAngle);
  }

  private double getFlightTime(Distance distanceToTarget) {
    // Simple linear approximation based on empirical data.
    return TIME_OF_FLIGHT_BY_DISTANCE.get(distanceToTarget.in(Meters));
  }

  private AngularVelocity calculateRequiredShooterSpeed(Distance distanceToTarget) {
    return RPM.of(SHOOTING_SPEED_BY_DISTANCE.get(distanceToTarget.in(Meters)));
  }

  private Angle calculateRequiredHoodAngle(Distance distanceToTarget) {
    return Degrees.of(HOOD_ANGLE_BY_DISTANCE.get(distanceToTarget.in(Meters)));
  }

  // meters, seconds
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_BY_DISTANCE =
      InterpolatingDoubleTreeMap.ofEntries(Map.entry(1.0, 1.0), Map.entry(4.86, 1.5));
  // Distance in meters, time in seconds
  // TODO: add more data points here.
  // CLOSE: NEED
  // MID: maybe good enough
  // FAR: NEED

  // meters, RPS
  private static final InterpolatingDoubleTreeMap SHOOTING_SPEED_BY_DISTANCE =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(Inches.of(88).in(Meters), 5800.0),
          Map.entry(Inches.of(101).in(Meters), 6300.0),
          Map.entry(Inches.of(133).in(Meters), 7050.0),
          Map.entry(Inches.of(166).in(Meters), 7900.0),
          Map.entry(Inches.of(209).in(Meters), 8600.0),
          Map.entry(Inches.of(118.51).in(Meters), 7500.0),
          Map.entry(Inches.of(111.51).in(Meters), 7000.0),
          Map.entry(Inches.of(97.51).in(Meters), 6800.0),
          Map.entry(Inches.of(87.51).in(Meters), 6600.0),
          Map.entry(Inches.of(77.51).in(Meters), 6100.0));

  // meters, degrees
  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_BY_DISTANCE =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.0, 15.0), Map.entry(2.0, 30.0), Map.entry(3.0, 45.0));
}
