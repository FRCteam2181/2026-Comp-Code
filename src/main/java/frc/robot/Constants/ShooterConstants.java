package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {
    
    public static final int kShooterLeader_ID = 9;
    public static final int kShooterFollower_ID = 14;

    public static final AngularVelocity kShooterVelocity = RPM.of(6000);
}
