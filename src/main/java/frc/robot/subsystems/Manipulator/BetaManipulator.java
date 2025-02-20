// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;

import frc.robot.BetaConstants;
import frc.robot.util.TalonPivot;

public class BetaManipulator extends BaseManipulator {
  private static BetaManipulator instance;

  public static BetaManipulator getInstance() {
      if (instance == null) {
          instance = new BetaManipulator();
      }

      return instance;
  }

  /** Creates a new BaseManipulator. */
  private BetaManipulator() {
    super(BetaConstants.Intake.SPIN.createTalon(),
         new TalonPivot(BetaConstants.Intake.PIVOT, 
         null, 
         BetaConstants.Intake.GEAR_RATIO, 
         BetaConstants.Intake.FF, 
         BetaConstants.Intake.ENCODER, 
         BetaConstants.Intake.TUNING_MODE, 
         "Beta Manipulator"), 
         BetaConstants.Intake.SPEED, 
         BetaConstants.Manipulator.BEAM_BREAK);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }
}
