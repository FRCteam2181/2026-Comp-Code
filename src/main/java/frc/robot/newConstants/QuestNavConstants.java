package frc.robot.newConstants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;

    public final class QuestNavConstants {
        public static Transform3d ROBOT_TO_QUEST = new Transform3d(
            new Translation3d(
                Inches.of(-10), 
                Inches.of(0), 
                Inches.of(11)), new Rotation3d(0,0,180));
    
        public static Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
              0.02, // Trust down to 2cm in Y direction
              0.035 // Trust down to 2 degrees rotational
        );
          public static final Distance FIELD_LENGTH = Meters.of(17.548);
  public static final Distance FIELD_WIDTH = Meters.of(8.052);
      }