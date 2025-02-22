// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;

import frc.robot.Constants;
import frc.robot.util.TalonPivot;

public class Manipulator extends TalonPivot {
  private static Manipulator instance;

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }
}
