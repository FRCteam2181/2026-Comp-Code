// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class DrivebaseConstants {
  // Hold time on motor brakes when disabled
  public static double WHEEL_LOCK_TIME = 10; // seconds

  public static double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static double MAX_SPEED = 7.597742314030; // in meters/second
  // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final Transform2d robotOffset =
      new Transform2d(
          Inches.of(17).in(Meters), Inches.of(0).in(Meters), Rotation2d.fromDegrees(180));

  public static final Transform2d climberOffset =
      new Transform2d(
          Inches.of(17.25).in(Meters), Inches.of(1).in(Meters), Rotation2d.fromDegrees(180));
  // bumpers are 3.75 inches thick
  // 12.5in from back of robot
  // 27/2 is 13.5
}
