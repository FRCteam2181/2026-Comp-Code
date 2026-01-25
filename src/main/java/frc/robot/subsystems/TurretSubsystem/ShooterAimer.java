package frc.robot.subsystems.TurretSubsystem;

import static edu.wpi.first.units.AngleUnit.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.math.*;

public class ShooterAimer extends SubsystemBase {

  double clearance;
  double velocity;
  double initialVelocity;
  double turretPitchAngle;
  double turretAngle;
  double turretAngleRelative;
  double hubRadius;
  double hubHeight;
  double hubInsideHeight;
  Transform3d ROBOT_TO_TURRET;
  Transform2d ROBOT_TO_TURRET_2D;
  Pose2d goalLocation;

  public ShooterAimer(Transform3d ROBOT_TO_TURRET) {
    clearance = Units.inchesToMeters(10); // clearance above goal (or smth idk)
    hubRadius = Units.inchesToMeters(23); // inches
    hubHeight = Units.inchesToMeters(72); // inches
    hubInsideHeight = Units.inchesToMeters(65); // inches
    this.ROBOT_TO_TURRET = ROBOT_TO_TURRET;
    ROBOT_TO_TURRET_2D =
        new Transform2d(this.ROBOT_TO_TURRET.getX(), this.ROBOT_TO_TURRET.getY(), new Rotation2d());
    goalLocation =
        new Pose2d(
            Units.inchesToMeters(651.2 - 158.6 - 47.0 / 2),
            Units.inchesToMeters(317.7 / 2),
            new Rotation2d());
  }

  public void findIdealVelocityAndAngle(Pose2d robotPose) {
    double x1 =
        new Pose3d(robotPose)
            .transformBy(ROBOT_TO_TURRET)
            .toPose2d()
            .getTranslation()
            .getDistance(goalLocation.getTranslation());
    // System.out.println("x1 = " + x1);
    double y1 = new Pose3d(robotPose).transformBy(ROBOT_TO_TURRET).getZ();
    double x2 = hubRadius;
    double y2 = hubHeight + clearance;
    double x3 = 0;
    double y3 = hubInsideHeight;
    System.out.println(
        "x1 = " + x1 + "\n" + "y1 = " + y1 + "\n" + "x2 = " + x2 + "\n" + "y2 = " + y2 + "\n"
            + "x3 = " + x3 + "\n" + "y3 = " + y3);
    double a =
        (y1 * (x2 - x3) + y2 * (-x1 + x3) + y3 * (x1 - x2)) / ((x1 - x2) * (x1 - x3) * (x2 - x3));
    double b =
        (-y1 * (x2 - x3) * (x2 + x3) + y2 * (x1 + x3) * (x1 - x3) - y3 * (x1 - x2) * (x1 + x2))
            / ((x1 - x2) * (x1 - x3) * (x2 - x3));
    // double c = (y1*x2*x3*(x2-x3)-y2*x1*x3*(x1-x3)+y3*x1*x2*(x1-x2))/((x1-x2)*(x1-x3)*(x2-x3));
    // double xIntercept = (-b-Math.sqrt(b^2-4*a*(c-y1)))/(2*a);
    turretPitchAngle = -Math.atan(2 * a * x1 + b);
    // velocity = (xIntercept-x1)/Math.cos(turretPitchAngle);
    initialVelocity = Math.sqrt((-9.81) / (2 * a * Math.pow(Math.cos(turretPitchAngle), 2)));

    System.out.println(
        "a = "
            + a
            + "\n"
            + "b = "
            + b
            + "\n"
            + "turretPitchAngle = "
            + turretPitchAngle
            + "\n"
            + "initialVelocity = "
            + initialVelocity);
  }

  public void update(
      Pose2d robotPose,
      ChassisSpeeds robotSpeed,
      TurretSubsystem turret,
      ShooterSubsystem shooter) {

    // 1. LATENCY COMP
    double latency = 0.15; // Tuned constant
    Pose2d futurePos =
        robotPose
            .transformBy(ROBOT_TO_TURRET_2D)
            .plus(
                new Transform2d(
                        robotSpeed.vxMetersPerSecond,
                        robotSpeed.vyMetersPerSecond,
                        new Rotation2d())
                    .times(latency));

    // 2. GET TARGET VECTOR

    Translation2d targetVec = goalLocation.getTranslation().minus(futurePos.getTranslation());
    double dist = targetVec.getNorm();

    // 3. CALCULATE IDEAL SHOT (Stationary)
    // Note: This returns HORIZONTAL velocity component
    // double idealHorizontalSpeed = ShooterTable.getSpeed(dist);
    findIdealVelocityAndAngle(robotPose);

    // 4. VECTOR SUBTRACTION
    Translation2d robotVelVec =
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    // Translation2d shotVec =
    // targetVec.div(dist).times(velocity*Math.cos(turretPitchAngle)).minus(robotVelVec);
    Translation2d shotVec = targetVec.minus(robotVelVec);

    // 5. CONVERT TO CONTROLS
    turretAngle = shotVec.getAngle().getDegrees();
    turretAngleRelative =
        Math.acos(
            (shotVec.dot(targetVec))
                / (shotVec.getNorm()
                    * targetVec
                        .getNorm())); // Angle relative to a straight line from the turret to the
    // hub (relative angle should be added counterclockwise)
    double newHorizontalSpeed = shotVec.getNorm();

    // 6. SOLVE FOR NEW PITCH/RPM
    velocity = newHorizontalSpeed / Math.cos(turretPitchAngle);
    // Clamp to avoid domain errors if we need more speed than possible
    // double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
    double newPitch = turretPitchAngle;

    // 7. SET OUTPUTS
    // turret.setAngle(turretAngle); //lowkenuinely don't know how to convert the units
    // hood.setAngle(Math.toDegrees(newPitch));
    // shooter.setVelocity(velocity); //idk how to implement
  }

  public void updateSim(Pose2d robotPose, ChassisSpeeds robotSpeed) {

    // 1. LATENCY COMP
    double latency = 0.15; // Tuned constant
    Pose2d futurePos =
        robotPose
            .transformBy(ROBOT_TO_TURRET_2D)
            .plus(
                new Transform2d(
                        robotSpeed.vxMetersPerSecond,
                        robotSpeed.vyMetersPerSecond,
                        new Rotation2d())
                    .times(latency));

    // 2. GET TARGET VECTOR

    Translation2d targetVec = goalLocation.getTranslation().minus(futurePos.getTranslation());
    double dist = targetVec.getNorm();

    // 3. CALCULATE IDEAL SHOT (Stationary)
    // Note: This returns HORIZONTAL velocity component
    // double idealHorizontalSpeed = ShooterTable.getSpeed(dist);
    findIdealVelocityAndAngle(robotPose);

    // 4. VECTOR SUBTRACTION
    Translation2d robotVelVec =
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    // Translation2d shotVec =
    // targetVec.div(dist).times(velocity*Math.cos(turretPitchAngle)).minus(robotVelVec);
    Translation2d shotVec = targetVec.minus(robotVelVec);

    // 5. CONVERT TO CONTROLS
    turretAngle = shotVec.getAngle().getRadians();
    turretAngleRelative =
        Math.acos(
            (shotVec.dot(targetVec))
                / (shotVec.getNorm()
                    * targetVec
                        .getNorm())); // Angle relative to a straight line from the turret to the
    // hub (relative angle should be added counterclockwise)
    double newHorizontalSpeed = shotVec.getNorm();

    // 6. SOLVE FOR NEW PITCH/RPM
    velocity = newHorizontalSpeed / Math.cos(turretPitchAngle);
    // Clamp to avoid domain errors if we need more speed than possible
    // double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
    double newPitch = turretPitchAngle;

    // 7. SET OUTPUTS
    // turnSim.setAngle(turretAngle); //lowkenuinely don't know how to convert the units
    // hoodSim.setAngle(Math.toDegrees(newPitch));
    // shootSim.setVelocity(velocity); //idk how to implement
  }

  public double getVelocity() {
    return velocity;
  }

  public double getTurretPitchAngle() {
    return turretPitchAngle;
  }

  public double getTurretAngle() {
    return turretAngle;
  }
}
