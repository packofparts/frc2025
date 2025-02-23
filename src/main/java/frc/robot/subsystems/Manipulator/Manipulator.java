// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;

import java.net.ContentHandler;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.TalonPivot;

public class Manipulator extends TalonPivot {
  private static Manipulator instance;
  private final TalonFX spin;
  private final DutyCycleOut control;

  public static Manipulator getInstance() {
      if (instance == null) {
          instance = new Manipulator();
      }

      return instance;
  }

  /** Creates a new BaseManipulator. */
  private Manipulator() {
    super(
      Constants.Manipulator.PIVOT_MOTOR, 
      null, 
      Constants.Manipulator.GEAR_RATIO, 
      Constants.Manipulator.FF, 
      Constants.Manipulator.ABSOLUTE_ENCODER, 
      Constants.Manipulator.TUNNING_MODE,
      "Manipulator"
    );
    spin = Constants.Manipulator.MANIPULATOR_MOTOR.createTalon();
    control = new DutyCycleOut(0.0);
  }

  public Command run(){
    return runOnce(() ->{
      // spin.setControl(control.withOutput(Constants.Manipulator.SPEED));
      spin.set(Constants.Manipulator.SPEED);
    });
  }

  public Command stop(){
    return runOnce(() ->{
      // spin.setControl(control.withOutput(0.0));
      spin.set(0.0);
    });
  }

  public Command reverse(){
    return runOnce(() ->{
      // spin.setControl(control.withOutput(-1 * Constants.Manipulator.SPEED));
      spin.set(-1 * Constants.Manipulator.SPEED);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }
}
