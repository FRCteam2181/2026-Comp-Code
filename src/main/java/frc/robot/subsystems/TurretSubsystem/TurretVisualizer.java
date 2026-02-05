// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.TurretSubsystem;

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
  private ShooterAimer shooterAimer;

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
    shooterAimer = new ShooterAimer(new Transform3d(0, 0, 0.451739, new Rotation3d()));
  }

  private Translation3d launchVel(LinearVelocity vel, Angle angle, Angle turretAngle) {
    Pose3d robot = poseSupplier.get();
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

    double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
    // System.out.println("horizontalVel = " + horizontalVel);
    double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
    // System.out.println("verticalVel = " + verticalVel);
    double xVel = horizontalVel * Math.cos(turretAngle.baseUnitMagnitude());
    // System.out.println("turretAngle.baseUnitMagnitude() = " + turretAngle.baseUnitMagnitude());
    // System.out.println("old xVel = " + xVel);
    double yVel = horizontalVel * Math.sin(turretAngle.baseUnitMagnitude());
    // System.out.println("old yVel = " + yVel);

    xVel += fieldSpeeds.vxMetersPerSecond;
    // System.out.println("new xVel = " + xVel);
    yVel += fieldSpeeds.vyMetersPerSecond;
    // System.out.println("new yVel = " + yVel);

    // System.out.println(
    //    "verticalVel manual = " + Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond));
    // System.out.println("fieldSpeeds.vyMetersPerSecond = " + fieldSpeeds.vyMetersPerSecond);
    // System.out.println("verticalVel = " + verticalVel);

    return new Translation3d(xVel, yVel, verticalVel);
  }

  public boolean canIntake() {
    return fuelStored < CAPACITY;
  }

  public void intakeFuel() {
    // int a = FuelSim.getInstance().handleIntakes(CAPACITY);
    // fuelStored += a;
    fuelStored++;
  }

  public void launchFuel(LinearVelocity vel, Angle angle, Angle turretAngle) {
    if (fuelStored == 0) return;
    // fuelStored--;
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
}
