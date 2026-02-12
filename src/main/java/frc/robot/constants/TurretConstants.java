package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {

  public static Translation2d ROBOT_TO_TURRET =
      new Translation2d(Inches.of(-6.25).in(Meters), Inches.of(6.25).in(Meters));

  public static Transform3d robotToTurret =
      new Transform3d(-0.19685, 0.0, 0.44, Rotation3d.kZero); // TODO add our variables
}
