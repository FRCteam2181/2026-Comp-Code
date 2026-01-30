package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {

  public static Translation2d ROBOT_TO_TURRET =
      new Translation2d(Inches.of(-6.25).in(Meters), Inches.of(6.25).in(Meters));
}
