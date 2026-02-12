package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import frc.robot.Robot;
import frc.robot.RobotState;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.telemetry.SmartMotorControllerTelemetryConfig;
import yams.units.EasyCRT;
import frc.robot.utils.FullSubsystem;
import yams.units.EasyCRTConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

/** Turret that uses YAMS CRT */
public class TurretSubsystem extends FullSubsystem {

    private static final double minAngle = Units.degreesToRadians(-210.0);
  private static final double maxAngle = Units.degreesToRadians(210.0);
  private static final double trackOverlapMargin = Units.degreesToRadians(10);
  private static final double trackCenterRads = (minAngle + maxAngle) / 2;
  private static final double trackMinAngle = trackCenterRads - Math.PI - trackOverlapMargin;
  private static final double trackMaxAngle = trackCenterRads + Math.PI + trackOverlapMargin;

  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Turret/MaxVelocity");
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Turret/MaxAcceleration", 9999999);
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA");

  /** Manually rerun CRT seeding. */
  private static final String RERUN_SEED = "Turret/CRT/RerunSeed";

  // private final TalonFX turretMotor;

  private final SparkFlex turretMotor = new SparkFlex(16, MotorType.kBrushless);
  private final AbsoluteEncoder cancoderB = turretMotor.getAbsoluteEncoder(); // 20t B SparkFlex
  private final DutyCycleEncoder cancoderA = new DutyCycleEncoder(0); // 19 A rio
  // Create a timer to delay CRT run until encoders are ready

  private Timer startUpTimer = new Timer();
  private boolean startTimer = false;
  private boolean delayForCRTDone = false;
  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig;
  private final SmartMotorControllerConfig motorConfig;
  private final SmartMotorController motor;
  private final MechanismPositionConfig robotToMechanism;
  private final PivotConfig pivotConfig;
  private final Pivot turret;
  private final Double absPositionASignal;
  private final Double absPositionBSignal;
  private final EasyCRTConfig easyCRTConfig;

  private boolean rotorSeededFromAbs = false;
  private double lastSeededTurretDeg = Double.NaN;
  private double lastSeedError = Double.NaN;
  private double lastAbsA = Double.NaN;
  private double lastAbsB = Double.NaN;
  private String lastSeedStatus = "NOT_ATTEMPTED";


   private LaunchState launchState = LaunchState.ACTIVE_LAUNCHING;
   private Rotation2d goalAngle = Rotation2d.kZero;
   private double goalVelocity = 0.0;
   private double lastGoalAngle = 0.0;

//   private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

//   private final Alert disconnected =
//       new Alert("Turret motor disconnected!", Alert.AlertType.kWarning);
//   @Setter private BooleanSupplier coastOverride = () -> false;

   TrapezoidProfile profile =
       new TrapezoidProfile(
           new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
   private State setpoint = new State();
   private double turretOffset;
   private boolean turretZeroed = false;

//   @Getter
//   @Accessors(fluent = true)
//   @AutoLogOutput
    private boolean atGoal = false;


   public TurretSubsystem() {

    absPositionASignal = (getAbsoluteEncoderWithOffset());
    absPositionBSignal = cancoderB.getPosition();

    motorTelemetryConfig =
        new SmartMotorControllerTelemetryConfig()
            .withMechanismPosition()
            .withRotorPosition()
            .withRotorVelocity()
            .withMechanismLowerLimit()
            .withMechanismUpperLimit();

    motorConfig =
        new SmartMotorControllerConfig(this)
            // .withClosedLoopController(.2, 0, 0)
            .withClosedLoopController(29.68, 0, 2.6489)
            .withSimClosedLoopController(2.596, 0, 0)
            .withSoftLimit(Rotations.of(-.3), Rotations.of(0.3))
            .withFeedforward(new SimpleMotorFeedforward(0.30397, 4.1323, 0.2806))
            .withSimFeedforward(new SimpleMotorFeedforward(0.45746, 2.1323, 2.2316))
            .withGearing(new MechanismGearing(GearBox.fromStages("4:1", "10:1")))
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("TurretMotorV2", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            // .withSupplyCurrentLimit(Amps.of(4))
            .withMotorInverted(true)
            .withControlMode(ControlMode.CLOSED_LOOP);

    motor = new SparkWrapper(turretMotor, DCMotor.getNeoVortex(1), motorConfig);

    robotToMechanism =
        new MechanismPositionConfig()
            .withMaxRobotHeight(Meters.of(1.5))
            .withMaxRobotLength(Meters.of(0.75))
            .withRelativePosition(
                new Translation3d(
                    Meters.of(0.0), // back from robot center
                    Meters.of(0.0), // centered left/right
                    Meters.of(0.451739) // up from the floor reference
                    ));

    pivotConfig =
        new PivotConfig(motor)
            .withHardLimit(Rotations.of(-.6), Rotations.of(0.6))
            .withTelemetry("Turret", TelemetryVerbosity.HIGH)
            .withStartingPosition(Degrees.of(0))
            .withMechanismPositionConfig(robotToMechanism)
            .withMOI(0.2);

    turret = new Pivot(pivotConfig);
    easyCRTConfig = buildEasyCrtConfig();
    logCrtConfigTelemetry();
    SmartDashboard.putBoolean(RERUN_SEED, false);
  }



  

  public Command sysId() {
    return turret.sysId(
        Volts.of(4.0), // maximumVoltage
        Volts.per(Second).of(0.5), // step
        Seconds.of(8.0) // duration
        );
  }

  /**
   * Set the dutycycle of the turret.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public Command setAngleDynamic(Supplier<Angle> angle) {
    return turret.setAngle(angle);
  }

  public Angle getRawAngle() {
    return turret.getAngle();
  }

  public double getPosition(){
return ((turret.getAngle()).in(Radians));
  }
  public Angle getRobotAdjustedAngle() {
    return turret.getAngle().plus(Degrees.of(180));
  }

  public double getRobotRelativeYawRadians() {
    return getRawAngle().in(edu.wpi.first.units.Units.Radians);
  }

  /** Forces a CRT reseed attempt */
  public void rerunCrtSeed() {
    rotorSeededFromAbs = false;
    SmartDashboard.putNumber("Turret/CRT/ManualRerunTimestampSec", Timer.getFPGATimestamp());
    attemptRotorSeedFromCANCoders();
  }

  public void periodic() {

    if (!startTimer) {
      startUpTimer.reset();
      startUpTimer.start();
      startTimer = true;
    }

    if (startUpTimer.hasElapsed(3)) {
      delayForCRTDone = true;
    }

    if (SmartDashboard.getBoolean(RERUN_SEED, false) && delayForCRTDone) {
      SmartDashboard.putBoolean(RERUN_SEED, false);
      rerunCrtSeed();
    }
    SmartDashboard.putNumber(
        "Turret/CRT/CurrentPositionDeg", motor.getMechanismPosition().in(Degrees));
    if (!rotorSeededFromAbs && delayForCRTDone) {
      attemptRotorSeedFromCANCoders();
    }
    turret.updateTelemetry();

    SmartDashboard.putNumber("Encoder A Raw", cancoderA.get());
    SmartDashboard.putNumber("Encoder A Adjusted", (getAbsoluteEncoderWithOffset()));
    SmartDashboard.putNumber("Encoder B", cancoderB.getPosition());
    SmartDashboard.putBoolean("Encoder A Raw", rotorSeededFromAbs);
    SmartDashboard.putNumber("Position", getRawAngle().in(Rotations));

   // Stop when disabled
    if (DriverStation.isDisabled() || !rotorSeededFromAbs) {
      atGoal = false;
    }

    // Update lastGoalAngle & reset setpoint
    if (DriverStation.isDisabled()) {
      setpoint = new State(inputs.positionRads, 0.0);
      lastGoalAngle = getPosition();
    }

    // Publish position
    if (rotorSeededFromAbs) {
      RobotState.getInstance()
          .addTurretObservation(
              new RobotState.TurretObservation(
                  Timer.getTimestamp(), new Rotation2d(getPosition())));
  }

    @Override
  public void periodicAfterScheduler() {
    // Delay tracking math until after the RobotState has been updated & turret zeroed
    if (DriverStation.isEnabled() && rotorSeededFromAbs) {
      Rotation2d robotAngle = RobotState.getInstance().getRotation();
      double robotAngularVelocity =
          RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond;

      Rotation2d robotRelativeGoalAngle = goalAngle.minus(robotAngle);
      double robotRelativeGoalVelocity = goalVelocity - robotAngularVelocity;

      boolean hasBestAngle = false;
      double bestAngle = 0;
      double minLegalAngle =
          switch (launchState) {
            case ACTIVE_LAUNCHING -> minAngle;
            case TRACKING -> trackMinAngle;
          };
      double maxLegalAngle =
          switch (launchState) {
            case ACTIVE_LAUNCHING -> maxAngle;
            case TRACKING -> trackMaxAngle;
          };
      for (int i = -2; i < 3; i++) {
        double potentialSetpoint = robotRelativeGoalAngle.getRadians() + Math.PI * 2.0 * i;
        if (potentialSetpoint < minLegalAngle || potentialSetpoint > maxLegalAngle) {
          continue;
        } else {
          if (!hasBestAngle) {
            bestAngle = potentialSetpoint;
            hasBestAngle = true;
          }
          if (Math.abs(lastGoalAngle - potentialSetpoint) < Math.abs(lastGoalAngle - bestAngle)) {
            bestAngle = potentialSetpoint;
          }
        }
      }
      lastGoalAngle = bestAngle;

      State goalState =
          new State(
              MathUtil.clamp(bestAngle, minLegalAngle, maxLegalAngle), robotRelativeGoalVelocity);

      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
      atGoal =
          EqualsUtil.epsilonEquals(bestAngle, setpoint.position)
              && EqualsUtil.epsilonEquals(robotRelativeGoalVelocity, setpoint.velocity);


      outputs.mode = TurretIOOutputMode.CLOSED_LOOP;
      outputs.position = setpoint.position - turretOffset;
      outputs.velocity = setpoint.velocity;
      outputs.kP = kP.get();
      outputs.kD = kD.get();
    }

    // Apply outputs
    turretIO.applyOutputs(outputs);
  }


  public void simulationPeriodic() {
    turret.simIterate();
  }

  /**
   * Tries to solve turret position via CRT and seed the relative encoder with the result. Reads
   * both CANCoder values, runs the solver, updates the SmartMotorController encoder, and publishes
   * CRT status to the dashboard.
   */
  private void attemptRotorSeedFromCANCoders() {
    AbsSensorRead absRead = readAbsSensors();
    if (!absRead.ok()) {
      if (!"NO_DEVICES".equals(absRead.status())) {
        SmartDashboard.putString("Turret/CRT/SeedStatus", absRead.status());
      }
      lastSeedStatus = absRead.status();
      return;
    }

    double absA = absRead.absA();
    double absB = absRead.absB();
    lastAbsA = absA;
    lastAbsB = absB;

    var solver = new EasyCRT(easyCRTConfig);
    var solvedAngle = solver.getAngleOptional();

    SmartDashboard.putNumber("Turret/CRT/AbsA", absA);
    SmartDashboard.putNumber("Turret/CRT/AbsB", absB);
    SmartDashboard.putString("Turret/CRT/SolverStatus", solver.getLastStatus());
    SmartDashboard.putNumber("Turret/CRT/SolverErrorRot", solver.getLastErrorRotations());
    SmartDashboard.putNumber("Turret/CRT/SolverIterations", solver.getLastIterations());

    if (solvedAngle.isEmpty()) {
      SmartDashboard.putBoolean("Turret/CRT/SolutionFound", false);
      lastSeedStatus = solver.getLastStatus();
      return;
    }

    Angle turretRotations = solvedAngle.get();
    motor.setEncoderPosition(turretRotations);
    rotorSeededFromAbs = true;
    lastSeededTurretDeg = turretRotations.in(Degrees);
    lastSeedError = solver.getLastErrorRotations();
    SmartDashboard.putBoolean("Turret/CRT/SolutionFound", true);
    SmartDashboard.putNumber("Turret/CRT/SeededTurretDeg", lastSeededTurretDeg);
    SmartDashboard.putNumber("Turret/CRT/MatchErrorRot", lastSeedError);

    lastSeedStatus = "OK";
    SmartDashboard.putString("Turret/CRT/SeedStatus", lastSeedStatus);
    SmartDashboard.putBoolean("Turret/CRT/Seeded", rotorSeededFromAbs);
  }

  /** Reads both absolute encoders and returns their rotations plus a status. */
  private AbsSensorRead readAbsSensors() {
    // Double absPositionASignal = (cancoderA.get());
    // Double absPositionBSignal = cancoderB.getPosition();

    boolean haveDevices = cancoderA != null && cancoderB != null;

    if (haveDevices) {

      return new AbsSensorRead(true, absPositionASignal, absPositionBSignal, "ok");
    }
    return new AbsSensorRead(false, Double.NaN, Double.NaN, "NO_DEVICES");
  }

  /** Build the CRT config */
  private EasyCRTConfig buildEasyCrtConfig() {
    // if (cancoderA.isConnected() && cancoderB.getPosition() > 0) {
    return new EasyCRTConfig(
            () -> Rotations.of(getAbsoluteEncoderWithOffset()),
            () -> Rotations.of(cancoderB.getPosition()))
        .withCommonDriveGear(1, 200, 19, 21)
        .withAbsoluteEncoderOffsets(Rotations.of(0), Rotations.of(-0.734803))
        .withAbsoluteEncoderInversions(false, false)
        .withMechanismRange(Rotations.of(-0.6), Rotations.of(0.6))
        .withMatchTolerance(Rotations.of(0.05))
        .withCrtGearRecommendationConstraints(1.2, 15, 60, 40);
    // } else {
    //   return null;
    // }
  }

  /** Publish CRT config-derived values for debugging coverage/ratios. */
  private void logCrtConfigTelemetry() {
    double mechanismRangeRot = easyCRTConfig.getMechanismRange().in(Rotations);
    double uniqueCoverageRot =
        easyCRTConfig.getUniqueCoverage().map(angle -> angle.in(Rotations)).orElse(Double.NaN);
    SmartDashboard.putNumber(
        "Turret/CRT/Config/RatioA", easyCRTConfig.getEncoder1RotationsPerMechanismRotation());
    SmartDashboard.putNumber(
        "Turret/CRT/Config/RatioB", easyCRTConfig.getEncoder2RotationsPerMechanismRotation());
    SmartDashboard.putNumber("Turret/CRT/Config/UniqueCoverageRot", uniqueCoverageRot);
    SmartDashboard.putBoolean(
        "Turret/CRT/Config/CoverageSatisfiesRange", easyCRTConfig.coverageSatisfiesRange());
    SmartDashboard.putNumber("Turret/CRT/Config/RequiredRangeRot", mechanismRangeRot);

    var configPair = easyCRTConfig.getRecommendedCrtGearPair();
    SmartDashboard.putBoolean("Turret/CRT/Config/RecommendedPairFound", configPair.isPresent());
    if (configPair.isPresent()) {
      var pair = configPair.get();
      SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedGearA", pair.gearA());
      SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedGearB", pair.gearB());
      SmartDashboard.putNumber(
          "Turret/CRT/Config/Reccomender/RecommendedCoverageRot", pair.coverage().in(Rotations));
      SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedLcm", pair.lcm());
      SmartDashboard.putBoolean(
          "Turret/CRT/Config/Reccomender/RecommendedCoprime",
          EasyCRTConfig.isCoprime(pair.gearA(), pair.gearB()));
      SmartDashboard.putNumber(
          "Turret/CRT/Config/Reccomender/RecommendedIterations", pair.theoreticalIterations());
    }
  }

  private Double getAbsoluteEncoderWithOffset() {

    return MathUtil.inputModulus(cancoderA.get() - ShooterConstants.EncoderAOffset, 0, 1);
  }

  private static record AbsSensorRead(boolean ok, double absA, double absB, String status) {}

    public enum LaunchState {
    ACTIVE_LAUNCHING,
    TRACKING
  }

}
