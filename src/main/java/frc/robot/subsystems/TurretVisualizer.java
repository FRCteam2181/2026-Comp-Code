// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.utils.FuelSim;
import java.util.function.Supplier;

// import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class TurretVisualizer {
  private Translation3d[] trajectory = new Translation3d[50];
  private Supplier<Pose3d> poseSupplier;
  private Supplier<ChassisSpeeds> fieldSpeedsSupplier;
  private final int CAPACITY = 30;
  public int fuelStored = 8;
  // private ShooterAimer shooterAimer;

  private final StructPublisher<Pose3d> turretVisualizerPublisher =
      NetworkTableInstance.getDefault()
          .getTable("turretVisualizer")
          .getStructTopic("fuelSimPose", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> velocityVectorPublisher =
      NetworkTableInstance.getDefault()
          .getTable("turretVisualizer")
          .getStructTopic("translation/velocityVector", Pose3d.struct)
          .publish();

  public TurretVisualizer(
      Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this.poseSupplier = poseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    // shooterAimer = new ShooterAimer(new Transform3d(0, 0, 0.451739, new Rotation3d()));
  }

  private Translation3d launchVel(LinearVelocity vel, Angle angle, Angle turretAngle) {
    Pose3d robot = poseSupplier.get();
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

    System.out.println("vel = " + vel);

    double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
    System.out.println("horizontalVel = " + horizontalVel);
    double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
    System.out.println("verticalVel = " + verticalVel);
    double xVel = horizontalVel * Math.cos(turretAngle.in(Radians));
    // System.out.println("turretAngle.baseUnitMagnitude() = " + turretAngle.baseUnitMagnitude());
    // System.out.println("old xVel = " + xVel);
    double yVel = horizontalVel * Math.sin(turretAngle.in(Radians));
    // System.out.println("old yVel = " + yVel);

    xVel += fieldSpeeds.vxMetersPerSecond;
    // System.out.println("new xVel = " + xVel);
    yVel += fieldSpeeds.vyMetersPerSecond;
    // System.out.println("new yVel = " + yVel);

    // System.out.println(
    //    "verticalVel manual = " + Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond));
    // System.out.println("fieldSpeeds.vyMetersPerSecond = " + fieldSpeeds.vyMetersPerSecond);
    // System.out.println("verticalVel = " + verticalVel);

    System.out.println(new Translation3d(xVel, yVel, verticalVel));

    return new Translation3d(xVel, yVel, verticalVel);
  }

  public boolean canIntake() {
    return fuelStored < CAPACITY;
  }

  public void intakeFuel() {
    int a = FuelSim.getInstance().getIntakedBalls();
    fuelStored += a;
    FuelSim.getInstance().clearIntakedBalls();
  }

  public void launchFuel(LinearVelocity vel, Angle angle, Angle turretAngle) {
    if (fuelStored == 0) return;
    fuelStored--;
    Pose3d robot = poseSupplier.get();

    Translation3d initialPosition =
        robot
            .getTranslation()
            .plus(
                new Translation3d(
                    0, // back from robot center
                    0, // centered left/right
                    0.451739 // up from the floor reference
                    ));
    FuelSim.getInstance().spawnFuel(initialPosition, launchVel(vel, angle, turretAngle));
    System.out.println(launchVel(vel, angle, turretAngle));
  }

  public void repeatedlyLaunchFuel() {
    // FuelSim.getInstance().spawnFuel(initialPosition, launchVel(vel, angle, turretAngle))
    //     .runOnce(() ->
    // launchFuel(ShooterAimer.getShotData(poseSupplier.get().toPose2d(),fieldSpeedsSupplier.get(),0).getVelocity(), ShooterAimer.getShotData(poseSupplier.get().toPose2d(),fieldSpeedsSupplier.get(),0).getPitchAngle(), ShooterAimer.getShotData(poseSupplier.get().toPose2d(),fieldSpeedsSupplier.get(),0).getAngle()))
    //     .andThen(Commands.waitSeconds(0.25))
    //     .repeatedly();
  }

  public void updateFuel(LinearVelocity vel, Angle angle, Angle turretAngle) {
    Translation3d trajVel = launchVel(vel, angle, turretAngle);
    velocityVectorPublisher.accept(
        poseSupplier.get().transformBy(new Transform3d(trajVel, new Rotation3d())));
    // System.out.println("trajVel = " + trajVel);
    for (int i = 0; i < trajectory.length; i++) {
      double t = i * 0.02;
      double step = 0.04 / 4;
      double x = trajVel.getX() * t + poseSupplier.get().getTranslation().getX();
      double y = trajVel.getY() * t + poseSupplier.get().getTranslation().getY();
      double z =
          trajVel.getZ() * t
              - 0.5 * 9.81 * t * t
              + poseSupplier.get().getTranslation().getZ()
              + 0.451739;

      trajectory[i] = new Translation3d(x, y, z);
    }

    // Logger.recordOutput("Turret/Trajectory", trajectory);
    // turretVisualizerPublisher.accept(trajectory);
    for (Translation3d trajectori : trajectory) {
      // System.out.println("\n" + trajectori);
      turretVisualizerPublisher.accept(new Pose3d(trajectori, new Rotation3d()));
    }
  }

  public void updateFuelDrag(LinearVelocity vel, Angle angle, Angle turretAngle) {
    Translation3d trajVel = launchVel(vel, angle, turretAngle);
    velocityVectorPublisher.accept(
        poseSupplier.get().transformBy(new Transform3d(trajVel, new Rotation3d())));
    // System.out.println("trajVel = " + trajVel);

    double v_h = trajVel.toTranslation2d().getNorm();
    double v_z = trajVel.getZ();
    double x = poseSupplier.get().getTranslation().getX();
    double y = poseSupplier.get().getTranslation().getY();

    Vectors vector = new Vectors(trajVel, new Translation3d(x, y, 0.451739), 0);

    // Runge-Kutta 4th Order Attempt
    for (int i = 0; i < trajectory.length; i++) {
      vector =
          runRK4(
              trajVel,
              Math.sqrt(
                  Math.pow(vector.getVelVec().getX(), 2) + Math.pow(vector.getVelVec().getY(), 2)),
              vector.getVelVec().getZ(),
              Math.sqrt(
                  Math.pow(vector.getPosVec().getX(), 2) + Math.pow(vector.getPosVec().getY(), 2)),
              vector.getPosVec().getZ());

      trajectory[i] =
          new Translation3d(
              vector.getPosVec().getX(), vector.getPosVec().getY(), vector.getPosVec().getZ());
    }

    // Logger.recordOutput("Turret/Trajectory", trajectory);
    // turretVisualizerPublisher.accept(trajectory);
    for (Translation3d trajectori : trajectory) {
      // System.out.println("\n" + trajectori);
      turretVisualizerPublisher.accept(new Pose3d(trajectori, new Rotation3d()));
    }
  }

  public Vectors runRK4(Translation3d trajVel, double v_h, double v_z, double h, double z) {
    double dt = 0.04;
    double step = dt;
    double c = 8;
    double m = 1;
    double g = 9.81;
    // dv_h/dt=(-c*sqrt((v_h)^2+(v_z)^2)*v_h)/m
    // dv_z/dt=-g-(c*sqrt((v_h)^2+(v_z)^2)*v_z)/m

    double k1_hv = (-c * Math.sqrt(Math.pow(v_h, 2) + Math.pow(v_z, 2)) * v_h) / m;
    double k1_zv = -g - (c * Math.sqrt(Math.pow(v_h, 2) + Math.pow(v_z, 2)) * v_h) / m;
    double k1_h = v_h;
    double k1_z = v_z;

    double k2_hv =
        (-c
                * Math.sqrt(Math.pow(v_h + k1_hv / 2, 2) + Math.pow(v_z + k1_zv / 2, 2))
                * (v_h + k1_hv / 2))
            / m;
    double k2_zv =
        -g
            - (c
                    * Math.sqrt(Math.pow(v_h + k1_hv / 2, 2) + Math.pow(v_z + k1_zv / 2, 2))
                    * (v_z + k1_zv / 2))
                / m;
    double k2_h = v_h + k1_hv;
    double k2_z = v_z + k1_zv;

    double k3_hv =
        (-c
                * Math.sqrt(Math.pow(v_h + k2_hv / 2, 2) + Math.pow(v_z + k2_zv / 2, 2))
                * (v_h + k2_hv / 2))
            / m;
    double k3_zv =
        -g
            - (c
                    * Math.sqrt(Math.pow(v_h + k2_hv / 2, 2) + Math.pow(v_z + k2_zv / 2, 2))
                    * (v_z + k2_zv / 2))
                / m;
    double k3_h = v_h + k2_hv;
    double k3_z = v_z + k2_zv;

    double k4_hv =
        (-c * Math.sqrt(Math.pow(v_h + k3_hv, 2) + Math.pow(v_z + k3_zv, 2)) * (v_h + k3_hv)) / m;
    double k4_zv =
        -g
            - (c * Math.sqrt(Math.pow(v_h + k3_hv, 2) + Math.pow(v_z + k3_zv, 2)) * (v_z + k3_zv))
                / m;
    double k4_h = v_h + k3_hv;
    double k4_z = v_z + k3_zv;

    double dv_h = step * (k1_hv + 2 * k2_hv + 2 * k3_hv + k4_hv) / 6;
    double dv_z = step * (k1_zv + 2 * k2_zv + 2 * k3_zv + k4_zv) / 6;
    double dh = step * (k1_h + 2 * k2_h + 2 * k3_h + k4_h) / 6;
    double dz = step * (k1_z + 2 * k2_z + 2 * k3_z + k4_z) / 6;

    v_h = v_h + dv_h;
    v_z = v_z + dv_z;
    h = h + dh;
    z = z + dz;

    double vx = v_h * Math.cos(trajVel.toTranslation2d().getAngle().getRadians());
    double vy = v_h * Math.sin(trajVel.toTranslation2d().getAngle().getRadians());
    double x = h * Math.cos(trajVel.toTranslation2d().getAngle().getRadians());
    double y = h * Math.sin(trajVel.toTranslation2d().getAngle().getRadians());
    return new Vectors(new Translation3d(vx, vy, v_z), new Translation3d(x, y, z), 0);
  }

  public void update3dPose(Angle azimuthAngle) {
    // Logger.recordOutput("Turret/TurretPose", new Pose3d(0, 0, 0, new Rotation3d(0, 0,
    // azimuthAngle.in(Radians))));
    // turretVisualizerPublisher.accept(
    //    new Pose3d(0, 0, 0, new Rotation3d(0, 0, azimuthAngle.in(Radians))));
  }

  /*@Override
  public void simulationPeriodic(){
    FuelSim.updateSim();
  }*/

  public record Vectors(Translation3d velVec, Translation3d posVec, int a) {
    public Vectors(Translation3d velVec, Translation3d posVec, double a) {
      this(velVec, posVec, (int) a);
    }

    public Translation3d getVelVec() {
      return velVec;
    }

    public Translation3d getPosVec() {
      return posVec;
    }

    @Override
    public final String toString() {
      return "velVec = " + velVec + ", posVec = " + posVec;
    }
  }
}
