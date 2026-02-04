// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.BottomIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TopIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem.*;
import frc.robot.utils.FuelSim;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Controllers and Button Board
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorControler = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing
  // selection of desired auto
  private final SendableChooser<Command> autoChooser;

  private final ClimberSubsystem climber = new ClimberSubsystem();

  private final TopIntakeSubsystem topintake = new TopIntakeSubsystem();
  private final BottomIntakeSubsystem bottomintake = new BottomIntakeSubsystem();

  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final TurretVisualizer turretVisualizer =
      new TurretVisualizer(() -> new Pose3d(drivebase.getPose()), drivebase::getFieldVelocity);
  private final ShooterAimer shooterAimer = new ShooterAimer(new Transform3d());

  private final SpindexerSubsystem spindexer = new SpindexerSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();
  private final InputSubsystem input = new InputSubsystem();
  public static Timer timerThing = new Timer();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1,
              () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
          .headingWhile(true);

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard =
      driveAngularVelocityKeyboard
          .copy()
          .withControllerHeadingAxis(
              () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
              () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(0));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    // Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard =
        drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard =
        drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard =
        drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    topintake.setDefaultCommand(topintake.set(0));
    bottomintake.setDefaultCommand(bottomintake.set(0));

    shooter.setDefaultCommand(shooter.set(0));

    spindexer.setDefaultCommand(spindexer.set(0));

    feeder.setDefaultCommand(feeder.set(0));
    input.setDefaultCommand(input.set(0));

    turret.setDefaultCommand(turret.set(0));

    if (Robot.isSimulation()) {
      // Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      /*driveDirectAngleKeyboard.driveToPose(
      () -> target,
      new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
      new ProfiledPIDController(
          5, 0, 0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));*/
      /*driverXbox
          .start()
          .onTrue(
              Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox
          .button(2)
          .whileTrue(
              Commands.runEnd(
                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));*/

      configureFuelSim();
      /*turretVisualizer.repeatedlyLaunchFuel(
      () -> LinearVelocity.ofBaseUnits(shooterAimer.getVelocity(), MetersPerSecond),
      () -> Angle.ofBaseUnits(shooterAimer.getTurretPitchAngle(), Radians),
      turret,
      shooter,
      shooterAimer);*/
      // driverXbox.button(2).onTrue(Commands.runOnce(() -> drivebase.launchFuel(), drivebase));
      driverXbox
          .button(2)
          .onTrue(
              Commands.runOnce(() -> timerThing.start())
                  .andThen(Commands.runOnce(() -> System.out.println("time = " + timerThing.get())))
                  .andThen(Commands.runOnce(() -> drivebase.launchFuel(), drivebase)));
      driverXbox
          .button(3)
          .onTrue(
              Commands.runOnce(() -> timerThing.start())
                  .andThen(Commands.runOnce(() -> System.out.println("time = " + timerThing.get())))
                  .andThen(Commands.runOnce(() -> drivebase.intakeFuel(), drivebase)));
    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(
          driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.y().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    }
    driverXbox.rightBumper().whileTrue(climber.c_climb());
    driverXbox.rightTrigger().whileTrue(climber.c_climbReverse());

    operatorControler
        .a()
        .whileTrue(
            topintake
                .set(IntakeConstants.kBottomIntakeDutyCycle)
                .alongWith(bottomintake.set(-IntakeConstants.kTopIntakeDutyCycle)));

    operatorControler.y().whileTrue(shooter.setVelocity(RPM.of(6350)));

    operatorControler
        .b()
        .whileTrue(spindexer.set(-.85).alongWith(feeder.set(-0.25).alongWith(input.set(.35))));

    operatorControler.rightTrigger().whileTrue(turret.set(.3));

    operatorControler.leftTrigger().whileTrue(turret.set(-.3));

    operatorControler.leftBumper().whileTrue(turret.sysId());
  }

  private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(
        Units.inchesToMeters(30), // Dimensions.FULL_WIDTH.in(Meters)
        Units.inchesToMeters(30), // Dimensions.FULL_LENGTH.in(Meters)
        Units.inchesToMeters(4), // Dimensions.BUMPER_HEIGHT.in(Meters)
        drivebase::getPose,
        drivebase::getFieldVelocity);
    instance.registerIntake(
        -Units.inchesToMeters(15), // -Dimensions.FULL_LENGTH.div(2).in(Meters),
        Units.inchesToMeters(15), // Dimensions.FULL_LENGTH.div(2).in(Meters),
        -Units.inchesToMeters(
            15 + 7), // -Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
        -Units.inchesToMeters(15), // -Dimensions.FULL_WIDTH.div(2).in(Meters),
        () -> true,
        () -> turretVisualizer.intakeFuel());

    instance.start();
    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  FuelSim.getInstance().clearFuel();
                  FuelSim.getInstance().spawnStartingFuel();
                })
            .withName("Reset Fuel")
            .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
