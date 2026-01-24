package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

// import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.ElevatorConstants;


public class RobotMath
{

  public static class ElevatorMath
  {

    /**
     * Convert {@link Distance} into {@link Angle}
     *
     * @param distance Distance, usually Meters.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertDistanceToRotations(Distance distance)
    {
      // m/(2*pi*r)*g = e
      return Rotations.of((distance.in(Meters) /
                          (Math.PI * ElevatorConstants.kElevatorDrumDiameter)*3) *
                          ElevatorConstants.kElevatorGearing);
    }

    /**
     * Convert {@link Angle} into {@link Distance}
     *
     * @param rotations Rotations of the motor
     * @return {@link Distance} of the elevator.
     */
    public static Distance convertRotationsToDistance(Angle rotations)
    {
      return Meters.of((rotations.in(Rotations) / ElevatorConstants.kElevatorGearing) *
                       ((Math.PI * ElevatorConstants.kElevatorDrumDiameter)*3));
    }
  }






 }
