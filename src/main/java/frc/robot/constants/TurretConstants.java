package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;

    public final class TurretConstants {
        public static Transform2d ROBOT_TO_TURRET = new Transform2d(
            new Translation2d(Inches.of((29.0 / 2) - 16.725), Inches.of((29.0 / 2.0) - 5.762)),
            Rotation2d.fromDegrees(180));
    

          public static final Distance FIELD_LENGTH = Meters.of(17.548);
  public static final Distance FIELD_WIDTH = Meters.of(8.052);
      }