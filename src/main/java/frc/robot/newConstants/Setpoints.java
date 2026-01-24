package frc.robot.newConstants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class Setpoints
{



  

  public static class AutoScoring
  {
    public static class Processor {
      public static final Transform2d offset = new Transform2d(Inches.of(18.5).in(Meters),
                                                               Inches.of(0).in(Meters),
                                                               Rotation2d.fromDegrees(180));
    }
    public static class Reef
    {

      public static final Transform2d coralOffset = new Transform2d(Inches.of(18.5).in(Meters),
                                                                    -Inches.of(1.5).in(Meters),
                                                                    Rotation2d.fromDegrees(180));
      public static final Transform2d algaeOffset = new Transform2d(Inches.of(18.5).in(Meters),
                                                                    Inches.of(4.97).in(Meters),
                                                                    Rotation2d.fromDegrees(180));
    }

    public static class HumanPlayer
    {

      public static class Left
      {

        public static final Transform2d offset = new Transform2d(Inches.of(18.5).in(Meters),
                                                                 Inches.of(0).in(Meters),
                                                                 Rotation2d.fromDegrees(0));
      }

      public static class Right
      {

        public static final Transform2d offset = new Transform2d(Inches.of(18.5).in(Meters),
                                                                 Inches.of(0).in(Meters),
                                                                 Rotation2d.fromDegrees(0));
      }
    }
  }


}
