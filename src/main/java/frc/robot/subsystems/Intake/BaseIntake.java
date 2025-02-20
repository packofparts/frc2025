// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import poplib.subsytems.pivot.Pivot;

public class BaseIntake extends SubsystemBase {
  protected Object spin;
  protected Pivot pivot;
  protected double spinSpeed;
  protected double maxError;
  
  /** Creates a new BaseIntake. */
  public BaseIntake(Object spinMotor, Pivot pivotSystem, double speed, double maxErrorValue) {
    spin = spinMotor;
    pivot = pivotSystem;
    spinSpeed = speed;
    maxError = maxErrorValue;
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

  public Command moveWrist(double position) {
    return pivot.moveWrist(position, maxError);
  }

  public void resetToAbsPosition(){
    pivot.resetToAbsolutePosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pivot.log();
  }
}
