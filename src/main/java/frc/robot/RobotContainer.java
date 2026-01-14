// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Subsystem Imports 

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.climberSubsystem;

import static edu.wpi.first.units.Units.RPM;
import java.io.File;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final ShooterSubsystem m_shooterSubsystem =new ShooterSubsystem();

  private final climberSubsystem m_ClimberSubsystem = new climberSubsystem();

  // Controllers and Button Board
  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController operatorController =new CommandXboxController(1);
  

  // The robot's subsystems and commands are defined here...

                                                                                


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */


  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */


  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */

  // Derive the heading axis with math!
  
  
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
   

    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.set(0));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    

    if (Robot.isSimulation())
    {
      //Pose2d target = new Pose2d(new Translation2d(1, 4),
                               //  Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      // Schedule `setVelocity` when the Xbox controller's B button is pressed,
    // cancelling on release.
    operatorController.a().whileTrue(m_shooterSubsystem.setVelocity(RPM.of(60)));
    operatorController.b().whileTrue(m_shooterSubsystem.setVelocity(RPM.of(-60)));
    //Schedule `set` when the Xbox controller's B button is pressed,
    // cancelling on release.
    operatorController.x().whileTrue(m_shooterSubsystem.set(0.3));
    operatorController.y().whileTrue(m_shooterSubsystem.set(-0.3));}

    driverXbox.y().whileTrue(m_ClimberSubsystem.c_climbReverse());
    driverXbox.x().whileTrue(m_ClimberSubsystem.c_climb());


  }



}
