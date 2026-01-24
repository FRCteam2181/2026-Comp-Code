// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.TurretSubsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class TurretIOSim extends SubsystemBase {

  public boolean turnMotorConnected = false;
  public Current turnCurrent = Amps.zero();
  public Angle turnPosition = Radians.zero();
  public AngularVelocity turnVelocity = RadiansPerSecond.zero();

  public boolean hoodMotorConnected = false;
  public Current hoodCurrent = Amps.zero();
  public Angle hoodPosition = Radians.zero();
  public AngularVelocity hoodVelocity = RadiansPerSecond.zero();

  public boolean flywheelMotorConnected = false;
  public Current flywheelCurrent = Amps.zero();
  public AngularVelocity flywheelSpeed = RadiansPerSecond.zero();

  public boolean shootMotorConnected = false;
  public Current shootCurrent = Amps.zero();
  public AngularVelocity shootSpeed = RadiansPerSecond.zero();

  private final DCMotor turnMotor = DCMotor.getKrakenX44Foc(1);
  private final SingleJointedArmSim turnSim =
      new SingleJointedArmSim(turnMotor, 2, 0.001, 0.1, 0, 2 * Math.PI, false, 0, 0.0, 0.0);

  private final DCMotor hoodMotor = DCMotor.getKrakenX44Foc(1);
  private final SingleJointedArmSim hoodSim =
      new SingleJointedArmSim(hoodMotor, 2, 0.005, 0.1, 0, Math.PI, false, 0, 0.0002, 0.0002);

  private final DCMotor flywheelMotor = DCMotor.getKrakenX60Foc(1);
  private final FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(flywheelMotor, 0.005, 2), flywheelMotor, 0.0005);

  private final DCMotor shootMotor = DCMotor.getKrakenX60Foc(1);
  private final FlywheelSim shootSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(shootMotor, 0.001, 2), shootMotor, 0.0005);

  public TurretIOSim() {
    setTurnOutput(Volts.zero());
    setHoodOutput(Volts.zero());
    setFlywheelOutput(Volts.zero());
    setShootOutput(Volts.zero());
  }

  // @Override
  public void updateInputs() {
    turnSim.update(0.02);
    hoodSim.update(0.02);
    flywheelSim.update(0.02);
    shootSim.update(0.02);
    // System.out.println(turnSim.getOutput(0));
    // System.out.println(turnSim.getOutput(1));

    // Simulate input values
    turnPosition = Radians.of(turnSim.getAngleRads());
    turnVelocity = RadiansPerSecond.of(turnSim.getVelocityRadPerSec());
    turnCurrent = Amps.of(turnSim.getCurrentDrawAmps());
    hoodPosition = Radians.of(hoodSim.getAngleRads());
    hoodVelocity = RadiansPerSecond.of(hoodSim.getVelocityRadPerSec());
    hoodCurrent = Amps.of(hoodSim.getCurrentDrawAmps());
    flywheelSpeed = flywheelSim.getAngularVelocity();
    flywheelCurrent = Amps.of(flywheelSim.getCurrentDrawAmps());
    shootSpeed = shootSim.getAngularVelocity();
    shootCurrent = Amps.of(shootSim.getCurrentDrawAmps());
  }

  // @Override
  public void setTurnOutput(Voltage out) {
    turnSim.setInput(out.in(Volts));
  }

  // @Override
  public void setHoodOutput(Voltage out) {
    hoodSim.setInput(out.in(Volts));
  }

  // @Override
  public void setFlywheelOutput(Voltage out) {
    flywheelSim.setInputVoltage(out.in(Volts));
  }

  // @Override
  public void setShootOutput(Voltage out) {
    shootSim.setInputVoltage(out.in(Volts));
  }
}
