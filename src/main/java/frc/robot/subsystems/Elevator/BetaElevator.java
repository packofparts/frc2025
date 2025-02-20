// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BetaConstants;
import frc.robot.util.TalonElevator;
import poplib.smart_dashboard.TunableNumber;

public class BetaElevator extends TalonElevator {
  private boolean resetSequence;
  private static BetaElevator instance;
  private final DigitalInput limitSwitch;
  private final TunableNumber setpoint;

  public static BetaElevator getInstance() {
    if (instance == null) {
        instance = new BetaElevator();
    }

    return instance;
}

  /** Creates a new BetaElevator. */
  public BetaElevator() {
    super(BetaConstants.Elevator.RIGHT_MOTOR, BetaConstants.Elevator.LEFT_MOTOR, BetaConstants.Elevator.FF_CONFIG, BetaConstants.Elevator.TUNNING_MODE, true, "Beta Elevator");
    limitSwitch = new DigitalInput(BetaConstants.Elevator.LIMIT_SWITCH_CHANNEL);
    resetSequence = false; 
    setpoint = new TunableNumber("Elevator Setpoint", BetaConstants.Elevator.SETPOINTS.IDLE.getSetpoint(), BetaConstants.Elevator.TUNNING_MODE); 
    tuning = BetaConstants.Elevator.RIGHT_MOTOR.genPIDTuning("Elevator Right Motor", BetaConstants.Elevator.TUNNING_MODE);
  }

  public Command moveElevator(double setPoint) {
        return run(() -> {
            setpoint.setDefault(setPoint);
            System.out.println("Running to : " + setPoint);
        })
        .until(() -> Math.abs(getEncoderPos() - setPoint) < BetaConstants.Elevator.MAX_ERROR);
  }

  public boolean isAtBottom() {
    return !limitSwitch.get();
  }

  public Command reZero() {
    return runOnce(() -> {
      resetSequence = true;
      moveDown(0.1);
      System.out.println("Reset Sequency Starting");
    }).andThen(run(() -> {
    })).until(() -> isAtBottom()).finallyDo(() -> {
      resetSequence = false;
      System.out.println("Reset Sequency Ending");
      stop();
      zeroPosition();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
