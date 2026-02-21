package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;

public class QuestNavConstants {
  public static Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          new Translation3d(Inches.of(6.001), Inches.of(-12.955), Inches.of(13.463)),
          new Rotation3d(180, 0, 180));

  public static Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.02, // Trust down to 2cm in X direction
          0.02, // Trust down to 2cm in Y direction
          0.035 // Trust down to 2 degrees rotational
          );
  public static Distance FIELD_LENGTH = Meters.of(17.548);
  public static Distance FIELD_WIDTH = Meters.of(8.052);
}
