// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootOnTheMoveCommandRevised;
import frc.robot.commands.ShootOnTheMoveCommandRevisedAdjusted;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OperatorConstants;
// import frc.robot.subsystems.BottomIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.InputSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TopIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.systems.GameData;
import frc.robot.systems.ScoringSystem;
import frc.robot.utils.controllerUtils.compBoardOne.CompBoardOne;
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
  // final CommandXboxController operatorControler = new CommandXboxController(1);
  final CompBoardOne compBoardOne;

  final CommandXboxController debugXbox = new CommandXboxController(4);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing
  // selection of desired auto
  private final SendableChooser<Command> autoChooser;

  private final ClimberSubsystem climber = new ClimberSubsystem();

  private final TopIntakeSubsystem topintake = new TopIntakeSubsystem();
  // private final BottomIntakeSubsystem bottomintake = new BottomIntakeSubsystem();

  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final IntakeArmSubsystem intakeArm = new IntakeArmSubsystem();
  private final SpindexerSubsystem spindexer = new SpindexerSubsystem();

  private final GameData gameData = new GameData(drivebase);

  // private final FeederSubsystem feeder = new FeederSubsystem();
  private final InputSubsystem input = new InputSubsystem();

  final ScoringSystem scoringSystem =
      new ScoringSystem(
          shooter,
          turret,
          drivebase,
          intakeArm,
          topintake,
          // bottomintake,
          spindexer,
          input); // intakeArm, climber, topintake, spindexer,

  // private final ArmSubsystem arm = new ArmSubsystem();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1.25,
              () -> driverXbox.getLeftX() * -1.25)
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
    this.compBoardOne = CompBoardOne.getInstance();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    NamedCommands.registerCommand(
        "Auto Target Then Shoot",
        new ParallelCommandGroup(
                new ShootOnTheMoveCommandRevised(drivebase, scoringSystem),
                // input.set(.7).alongWith(new WaitCommand(.25).andThen(spindexer.set(.8))))
                scoringSystem.runInputAndIdexerForwards(3500, .9).withTimeout(4))
            .withTimeout(5));
    // shooter.setVelocity(RPM.of(500)).withTimeout(5));

    NamedCommands.registerCommand(
        "Localize", Commands.runOnce(() -> drivebase.resetAutoBuilderOdometry(), drivebase));

    // NamedCommands.registerCommand(
    //     "Intake Down",
    //     topintake
    //         .set(IntakeConstants.kBottomIntakeDutyCycle)
    //         // .alongWith(bottomintake.set(IntakeConstants.kTopIntakeDutyCycle))
    //         .withTimeout(5));
    NamedCommands.registerCommand("Intake Down", scoringSystem.armDown(-.35).withTimeout(.5));

    NamedCommands.registerCommand("Intake Up", scoringSystem.armUp(-.35).withTimeout(.5));

    NamedCommands.registerCommand(
        "Run Intake",
        scoringSystem.runIntakeForwards(
            IntakeConstants.kBottomIntakeDutyCycle, IntakeConstants.kTopIntakeDutyCycle));

    NamedCommands.registerCommand(
        "Intake Down + Start",
        scoringSystem.intakeSetAndStart(Angle.ofBaseUnits(2, Degrees), 0.5, 0.5));
    NamedCommands.registerCommand(
        "Intake Up + Stop",
        scoringSystem.intakeSetAndStart(Angle.ofBaseUnits(90, Degrees), 0, 0).withTimeout(0.5));
    NamedCommands.registerCommand("Stop Commands", scoringSystem.stopAllCommand());

    NamedCommands.registerCommand(
        "Stop Input and Feeder", scoringSystem.runInputAndIdexerForwards(0, 0).withTimeout(4));

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    // Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putNumber("ShootSpeed", 100);
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
    // bottomintake.setDefaultCommand(bottomintake.set(0));

    shooter.setDefaultCommand(shooter.set(0));

    spindexer.setDefaultCommand(spindexer.set(0));

    // feeder.setDefaultCommand(feeder.set(0));
    input.setDefaultCommand(input.set(0));

    turret.setDefaultCommand(turret.set(0));

    intakeArm.setDefaultCommand(intakeArm.set(0));

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(
          () -> target,
          new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
          new ProfiledPIDController(
              5, 0, 0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
    }

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.y().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    driverXbox
        .rightBumper()
        .whileTrue(
            climber
                .c_climb()
                .until(climber.hitForwrdLimit())
                .andThen(new RunCommand(() -> driverXbox.setRumble(RumbleType.kBothRumble, 1))))
        .onFalse(new RunCommand(() -> driverXbox.setRumble(RumbleType.kBothRumble, 0)));

    driverXbox
        .leftBumper()
        .whileTrue(
            climber
                .c_climbReverse()
                .until(climber.hitReverseLimit())
                .andThen(new RunCommand(() -> driverXbox.setRumble(RumbleType.kBothRumble, 1))))
        .onFalse(new RunCommand(() -> driverXbox.setRumble(RumbleType.kBothRumble, 0)));
    driverXbox.x().onTrue(Commands.runOnce(() -> drivebase.photonOverride(), drivebase));

    // driverXbox.leftTrigger().whileTrue(Commands.runOnce(() -> turret.rezeroTurretPosition()));

    driverXbox.button(8).whileTrue(drivebase.driveToPose(() -> scoringSystem.getClimbPoseRight()));

    driverXbox.button(7).whileTrue(drivebase.driveToPose(() -> scoringSystem.getClimbPoseLeft()));

    driverXbox
        .b()
        .onTrue(
            Commands.runOnce(() -> drivebase.setQuestNavPose(drivebase.getPose3d()), drivebase)
                .ignoringDisable(true));

    // driverXbox.button(9).onTrue(Commands.runOnce(() ->
    // {driveAngularVelocity.scaleTranslation(1);}));
    // driverXbox.button(9).onFalse(Commands.runOnce(() ->
    // {driveAngularVelocity.scaleTranslation(.8);}));

    // Buttonboard Buttons

    // 1. Reverse shooter
    compBoardOne.CompBoardOneButtonA().whileTrue(scoringSystem.setShooterRPMReverse(5500));

    // 2. Reverse intake
    compBoardOne
        .CompBoardOneButtonB()
        .whileTrue(
            scoringSystem.runIntakeReverse(
                IntakeConstants.kBottomIntakeDutyCycle, IntakeConstants.kTopIntakeDutyCycle));

    // 3. Reverse spindexer and input
    compBoardOne.CompBoardOneButtonC().whileTrue(scoringSystem.runInputAndIdexerReverse(.65, .85));

    // // 4. light show
    // // // compBoardOne.CompBoardOneButtonD().whileTrue();

    // 5. Turret dutycycle left
    compBoardOne.CompBoardOneButtonL1().whileTrue(scoringSystem.turnTurretLeft(.2));

    // 6. Turret dutycycle right
    compBoardOne.CompBoardOneButtonR1().whileTrue(scoringSystem.turnTurretRight(.2));

    // 7. AutoAim
    // driverXbox
    // .y()
    compBoardOne
        .CompBoardOneButtonL2()
        .toggleOnTrue(
            new ShootOnTheMoveCommandRevisedAdjusted(drivebase, scoringSystem)
                .withName("OperatorControls.aimCommand"));

    // // 8. Run spindexer+input
    compBoardOne
        .CompBoardOneButtonR2()
        // .toggleOnTrue(new RunCommand(() -> scoringSystem.runInputAndIdexerAtShooterSpeed()))
        // .onFalse(scoringSystem.runInputAndIdexerForwards(0, 0));
        .whileTrue(scoringSystem.runInputAndIdexerForwards(3500, .95));
    // .alongWith(scoringSystem.runInputAndIdexerAtShooterSpeed()));

    // Commands.runOnce(() -> scoringSystem.setInputVelocitySetpoint())))
    // .onFalse(input.set(0));

    // // 9. Run spindexer+input w/ arm agitation
    // compBoardOne.CompBoardOneButtonSelect().whileTrue(scoringSystem.useArmToAgitate().repeatedly());

    compBoardOne.CompBoardOneButtonSelect().whileTrue(scoringSystem.armUp(.60));

    // 10. hood up
    // compBoardOne
    //     .CompBoardOneButtonStart()
    //     // driverXbox
    //     //     .x()
    //     .toggleOnTrue(new ShootOnTheMoveCommandRevisedAdjusted(drivebase, scoringSystem,
    // "Left"));
    // WARNING this button is temporarily porgrammed to run driveToPose for climbing and is
    // UNTESTED
    // // on the real robot
    // compBoardOne
    //     .CompBoardOneButtonStart()
    //     .whileTrue(drivebase.driveToPose(() -> scoringSystem.getClimbPose()));
    // compBoardOne.CompBoardOneButtonStart().whileTrue(intakeArm.setAngle(Degrees.of(-100)));

    // 11. hood down

    // TEMP SysID for arm
    // compBoardOne.CompBoardOneButtonL3().whileTrue(intakeArm.setAngle(Degrees.of(0)));
    // compBoardOne
    //     .CompBoardOneButtonL3()
    //     // driverXbox
    //     //     .b()
    //     .toggleOnTrue(new ShootOnTheMoveCommandRevisedAdjusted(drivebase, scoringSystem,
    // "Right"));

    // TEMP run spindexer and input at velocity
    // WARNING this button is temporarily porgrammed to run the intake and spindexer based on RPM
    // related to the shooter's speed and is UNTESTED on the real robot
    // shooter must be running using the autoaim, otherwise will return 0 for both
    // compBoardOne.CompBoardOneButtonL3().whileTrue(scoringSystem.runInputAndIdexerAtShooterSpeed());

    // 12. intake up
    compBoardOne.CompBoardOneButtonR3().whileTrue(scoringSystem.armUp(-.45));

    // 13. intake down
    compBoardOne.CompBoardOneJoystickAsButtonNegX().whileTrue(scoringSystem.armDown(-.35));

    // 14. run intake
    compBoardOne
        .CompBoardOneJoystickAsButtonPosX()
        .toggleOnTrue(
            scoringSystem.runIntakeForwards(
                IntakeConstants.kBottomIntakeDutyCycle, IntakeConstants.kTopIntakeDutyCycle));

    // 15. shooter shoot
    compBoardOne
        .CompBoardOneJoystickAsButtonNegY()
        .whileTrue(scoringSystem.setShooterRPMForwards(6500));

    // Debug stuff, only when xbox is in port 4
    debugXbox
        .a()
        .onTrue(
            new RunCommand(
                () -> {
                  climber.climberRelative.setPosition(140);
                }));
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
