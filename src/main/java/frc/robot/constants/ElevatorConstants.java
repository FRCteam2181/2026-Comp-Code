// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ElevatorConstants{
        public static final double   kElevatorKp              = 22;
        public static final double   kElevatorKi              = 0.5;
        public static final double   kElevatorKd              = 1.5;
        
        public static final double   kElevatorkS              = 0;//0.01964; // volts (V)
        public static final double   kElevatorkV              = 0;//2.63; // volt per velocity (V/(m/s))
        public static final double   kElevatorkA              = 0;//0.14; // volt per acceleration (V/(m/s²))
        public static final double   kElevatorkG              = 0;//0.91274; // volts (V)
       
        public static final double   kElevatorGearing         = 12.0; 
        public static final double   kElevatorDrumDiameter      = Units.inchesToMeters(1.751);
       
        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        public static final double   kMinElevatorHeightMeters = 0;//min height 
        public static final double   kMaxElevatorHeightMeters = 10.25;
        public static final Distance kMinElevatorHeight      = Meters.of(kMinElevatorHeightMeters);
        public static final Distance kMaxElevatorHeight      = Meters.of(kMaxElevatorHeightMeters);
        public static final double   kElevatorAllowableError = .04;  //.04
        public static final double   kLowerToScoreHeight     = Units.inchesToMeters(6);
        
        public static       double   kElevatorRampRate       = 0.1;
        public static       int      kElevatorCurrentLimit   = 40;
        public static double kMaxVelocity = Meters.of(13).per(Second).in(MetersPerSecond);
        public static double kMaxAcceleration = Meters.of(13).per(Second).per(Second).in(MetersPerSecondPerSecond);
        public static final double   kElevatorUnextendedHeight    = Units.inchesToMeters(41.5);

        public static final double k_FeederStation = Units.inchesToMeters(17);//17.375
        public static final double k_L1 = Units.inchesToMeters(0);
        public static final double k_L2 = Units.inchesToMeters(27.75);
        public static final double k_L3 = Units.inchesToMeters(43.625);
        public static final double k_L4 = Units.inchesToMeters(68.875);

        public static final double k_Processor = 0;
        public static final double k_AGround = 0;
        public static final double k_A1 = Units.inchesToMeters(15.5);
        public static final double k_A2 = Units.inchesToMeters(31);
        public static final double k_Net = Units.inchesToMeters(73.875);
        public static final double k_L4BumpUP = Units.inchesToMeters(73.5);
        
        public static final int k_ElevatorLeftID = 17;
        public static final int k_ElevatorRightID = 16;

        public static final Distance kLaserCANOffset          = Meters.of(0);//TODO Change
        public static final int elevatorLaserCanID  = 19;
      }