// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaConstants;
import poplib.sensors.beam_break.BeamBreak;
import poplib.sensors.beam_break.BeamBreakConfig;
import poplib.subsytems.pivot.Pivot;

public class BaseManipulator extends SubsystemBase {
  protected Object spin;
  protected Pivot pivot;
  protected double spinSpeed;
  protected BeamBreak beamBreak;

  /** Creates a new BaseManipulator. */
  public BaseManipulator(Object spinMotor, Pivot pivotSystem, double speed, BeamBreakConfig beamBreakConfig) {
    spin = spinMotor;
    pivot = pivotSystem;
    spinSpeed = speed;
    beamBreak = beamBreakConfig.createBeamBreak();
  }

  public boolean coralIn() {
    return beamBreak.isBlocked();
  }

  public Command run() {
    return runOnce(() -> {
      if (spin instanceof SparkMax){
        ((SparkMax) spin).set(spinSpeed);
      }
      else if (spin instanceof TalonFX){
        ((TalonFX) spin).set(spinSpeed);
      }
    });
  }

  public Command stop() {
    return runOnce(() -> {
      if (spin instanceof SparkMax){
        ((SparkMax) spin).set(0.0);
      }
      else if (spin instanceof TalonFX){
        ((TalonFX) spin).set(0.0);
      }
    });
  }

  public Command reverse() {
    return runOnce(() -> {
      if (spin instanceof SparkMax){
        ((SparkMax) spin).set(-1 * spinSpeed);
      }
      else if (spin instanceof TalonFX){
        ((TalonFX) spin).set(-1 * spinSpeed);
      }
    });
  }

  public Command intakeCoral(){
    return run(() -> {
      if (spin instanceof SparkMax){
        ((SparkMax) spin).set(-1 * spinSpeed);
      }
      else if (spin instanceof TalonFX){
        ((TalonFX) spin).set(-1 * spinSpeed);
      }
    }).until(beamBreak.getBlockedSupplier()).andThen(stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pivot.log();
    SmartDashboard.putBoolean("Coral In", coralIn());
  }
}
