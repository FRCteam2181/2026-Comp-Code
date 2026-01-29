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
    clearance = Units.inchesToMeters(21); // clearance above goal (or smth idk)
    hubRadius = Units.inchesToMeters(24); // inches
    hubHeight = Units.inchesToMeters(72); // inches
    hubInsideHeight = Units.inchesToMeters(48); // inches
    this.ROBOT_TO_TURRET = ROBOT_TO_TURRET;
    ROBOT_TO_TURRET_2D =
        new Transform2d(this.ROBOT_TO_TURRET.getX(), this.ROBOT_TO_TURRET.getY(), new Rotation2d());
    goalLocation =
        new Pose2d(
            Units.inchesToMeters(651.2 - 158.6 - 47.0 / 2),
            Units.inchesToMeters(317.7 / 2),
            new Rotation2d());
  }

  public void findIdealVelocityAndAngle(Pose2d robotPose, ChassisSpeeds robotSpeed) {
    double x1 =
        -1
            * new Pose3d(robotPose)
                .transformBy(ROBOT_TO_TURRET)
                .toPose2d()
                .getTranslation()
                .getDistance(goalLocation.getTranslation());
    // System.out.println("x1 = " + x1);
    double y1 = new Pose3d(robotPose).transformBy(ROBOT_TO_TURRET).getZ();
    double x2 = -1 * hubRadius;
    double y2 = hubHeight + clearance;
    double x3 = 0;
    double y3 = hubInsideHeight;
    System.out.println(
        "x1 = " + x1 + "\n" + "y1 = " + y1 + "\n" + "x2 = " + x2 + "\n" + "y2 = " + y2 + "\n"
            + "x3 = " + x3 + "\n" + "y3 = " + y3);
    double a =
        (y1 * (x2 - x3) + y2 * (-x1 + x3) + y3 * (x1 - x2)) / ((x1 - x2) * (x1 - x3) * (x2 - x3));
    // double a=(y1*(x2-x3)+y2*(-x1+x3)+y3*(x1-x2))/((x1-x2)*(x1-x3)*(x2-x3))
    double b =
        (-y1 * (x2 - x3) * (x2 + x3) + y2 * (x1 + x3) * (x1 - x3) - y3 * (x1 - x2) * (x1 + x2))
            / ((x1 - x2) * (x1 - x3) * (x2 - x3));
    // double c = (y1*x2*x3*(x2-x3)-y2*x1*x3*(x1-x3)+y3*x1*x2*(x1-x2))/((x1-x2)*(x1-x3)*(x2-x3));
    // double xIntercept = (-b-Math.sqrt(b^2-4*a*(c-y1)))/(2*a);

    // Distance FIELD_LENGTH = Inches.of(650.12);
    // Distance FIELD_WIDTH = Inches.of(316.64);
    // ShotData calculatedShot =
    //     iterativeMovingShotFromFunnelClearance(
    //         robotPose,
    //         robotSpeed,
    //         new Translation3d(
    //             Inches.of(650.12).minus(Inches.of(181.56)),
    //             Inches.of(316.64).div(2),
    //             Inches.of(56.4)),
    //         3);

    turretPitchAngle = Math.atan(2 * a * x1 + b);
    // turretPitchAngle = calculatedShot.getHoodAngle().magnitude();
    // initialVelocity = calculatedShot.getExitVelocity().magnitude();
    // System.out.println(Math.atan(2 * a * x1 + b));
    // System.out.println(Math.sqrt((-9.81) / (2 * a * Math.pow(Math.cos(turretPitchAngle), 2))));
    // System.out.println(calculatedShot.getTarget());
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
    findIdealVelocityAndAngle(robotPose, robotSpeed);

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
    findIdealVelocityAndAngle(robotPose, robotSpeed);

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
    // velocity = newHorizontalSpeed / Math.cos(turretPitchAngle);
    velocity = initialVelocity;
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

  //   public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
  //     return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
  //   }

  //   // see https://www.desmos.com/geometry/l4edywkmha
  //   public static Angle calculateAngleFromVelocity(
  //       Pose2d robot, LinearVelocity velocity, Translation3d target) {
  //     double g = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);
  //     double vel = velocity.in(InchesPerSecond);
  //     double x_dist = getDistanceToTarget(robot, target).in(Inches);
  //     double y_dist =
  //         target
  //             .getMeasureZ()
  //             .minus(
  //                 new Transform3d(
  //                         0, // back from robot center
  //                         0, // centered left/right
  //                         0.451739, // up from the floor reference
  //                         new Rotation3d())
  //                     .getMeasureZ())
  //             .in(Inches);
  //     double angle =
  //         Math.atan(
  //             ((vel * vel)
  //                     + Math.sqrt(
  //                         Math.pow(vel, 4) - g * (g * x_dist * x_dist + 2 * y_dist * vel * vel)))
  //                 / (g * x_dist));
  //     return Radians.of(angle);
  //   }

  //   // calculates how long it will take for a projectile to travel a set distance given its
  // initial
  //   // velocity and angle
  //   public static Time calculateTimeOfFlight(
  //       LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
  //     double vel = exitVelocity.in(MetersPerSecond);
  //     double angle = hoodAngle.in(Radians);
  //     double dist = distance.in(Meters);
  //     return Seconds.of(dist / (vel * Math.cos(angle)));
  //   }

  //   public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
  //     return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
  //   }

  //   public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
  //     return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
  //   }

  //   // calculates the angle of a turret relative to the robot to hit a target
  //   public static Angle calculateAzimuthAngle(Pose2d robot, Translation3d target) {
  //     Translation2d turretTranslation =
  //         new Pose3d(robot)
  //             .transformBy(
  //                 new Transform3d(
  //                     0, // back from robot center
  //                     0, // centered left/right
  //                     0.451739, // up from the floor reference
  //                     new Rotation3d()))
  //             .toPose2d()
  //             .getTranslation();

  //     Translation2d direction = target.toTranslation2d().minus(turretTranslation);

  //     return Radians.of(
  //         MathUtil.inputModulus(
  //             direction.getAngle().minus(robot.getRotation()).getRadians(), 0, 2 * Math.PI));
  //   }

  //   // Move a target a set time in the future along a velocity defined by fieldSpeeds
  //   public static Translation3d predictTargetPos(
  //       Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
  //     double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond *
  // timeOfFlight.in(Seconds);
  //     double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond *
  // timeOfFlight.in(Seconds);

  //     return new Translation3d(predictedX, predictedY, target.getZ());
  //   }

  //   // Custom velocity ramp meant to minimize how fast the flywheels have to change speed
  //   public static LinearVelocity scaleLinearVelocity(Distance distanceToTarget) {
  //     double VEL_MULTIPLIER = 70.0; // multiplies goal velocity for targetting
  //     double VEL_POWER = 0.3; // raises goal velocity to power
  //     LinearVelocity BASE_VEL = InchesPerSecond.of(50); // added to final velocity
  //     double velocity =
  //         BASE_VEL.in(InchesPerSecond)
  //             + VEL_MULTIPLIER * Math.pow(distanceToTarget.in(Inches), VEL_POWER);
  //     return InchesPerSecond.of(velocity);
  //   }

  //   // see https://www.desmos.com/calculator/ezjqolho6g
  //   public static ShotData calculateShotFromFunnelClearance(
  //       Pose2d robot, Translation3d actualTarget, Translation3d predictedTarget) {
  //     double VEL_MULTIPLIER = 70.0; // multiplies goal velocity for targetting
  //     double VEL_POWER = 0.3; // raises goal velocity to power
  //     LinearVelocity BASE_VEL = InchesPerSecond.of(50); // added to final velocity
  //     Distance DISTANCE_ABOVE_FUNNEL = Inches.of(20); // how high to clear the funnel
  //     Distance FUNNEL_RADIUS = Inches.of(24);
  //     Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);

  //     double x_dist = getDistanceToTarget(robot, predictedTarget).in(Inches);
  //     double y_dist =
  //         predictedTarget
  //             .getMeasureZ()
  //             .minus(
  //                 new Transform3d(
  //                         0, // back from robot center
  //                         0, // centered left/right
  //                         0.451739, // up from the floor reference
  //                         new Rotation3d())
  //                     .getMeasureZ())
  //             .in(Inches);
  //     double g = 386;
  //     double r =
  //         FUNNEL_RADIUS.in(Inches) * x_dist / getDistanceToTarget(robot,
  // actualTarget).in(Inches);
  //     double h = FUNNEL_HEIGHT.plus(DISTANCE_ABOVE_FUNNEL).in(Inches);
  //     double A1 = x_dist * x_dist;
  //     double B1 = x_dist;
  //     double D1 = y_dist;
  //     double A2 = -x_dist * x_dist + (x_dist - r) * (x_dist - r);
  //     double B2 = -r;
  //     double D2 = h;
  //     double Bm = -B2 / B1;
  //     double A3 = Bm * A1 + A2;
  //     double D3 = Bm * D1 + D2;
  //     double a = D3 / A3;
  //     double b = (D1 - A1 * a) / B1;
  //     double theta = Math.atan(b);
  //     double v0 = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));

  //     System.out.println(
  //         "a = " + a + "\n" + "b = " + b + "\n" + "theta = " + theta + "\n" + "v0 = " + v0 +
  // "\n");
  //     return new ShotData(InchesPerSecond.of(v0), Radians.of(theta), predictedTarget);
  //   }

  //   // use an iterative lookahead approach to determine shot parameters for a moving robot
  //   public static ShotData iterativeMovingShotFromFunnelClearance(
  //       Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
  //     // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
  //     ShotData shot = calculateShotFromFunnelClearance(robot, target, target);
  //     Distance distance = getDistanceToTarget(robot, target);
  //     Time timeOfFlight =
  //         calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
  //     Translation3d predictedTarget = target;

  //     // Iterate the process, getting better time of flight estimations and updating the
  // predicted
  //     // target accordingly
  //     for (int i = 0; i < iterations; i++) {
  //       predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
  //       shot = calculateShotFromFunnelClearance(robot, target, predictedTarget);
  //       timeOfFlight =
  //           calculateTimeOfFlight(
  //               shot.getExitVelocity(),
  //               shot.getHoodAngle(),
  //               getDistanceToTarget(robot, predictedTarget));
  //     }

  //     return shot;
  //   }

  //   public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {
  //     public ShotData(LinearVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
  //       this(exitVelocity.in(MetersPerSecond), hoodAngle.in(Radians), target);
  //     }

  //     public LinearVelocity getExitVelocity() {
  //       return MetersPerSecond.of(this.exitVelocity);
  //     }

  //     public Angle getHoodAngle() {
  //       return Radians.of(this.hoodAngle);
  //     }

  //     public Translation3d getTarget() {
  //       return this.target;
  //     }
  //   }
}
