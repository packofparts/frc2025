// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import poplib.smart_dashboard.TunableNumber;
import poplib.subsytems.elevator.SparkElevator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


public class Elevator extends SparkElevator {
    private final DigitalInput limitSwitch;
    private boolean resetSequence;
    private static Elevator instance;
    private TunableNumber scoringPos;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private Elevator() { 
        super(Constants.Elevator.RIGHT_MOTOR, Constants.Elevator.LEFT_MOTOR, 
        Constants.Elevator.FF_CONFIG, Constants.Elevator.TUNNING_MODE, true, "elevator");   
        
        scoringPos = new TunableNumber("Elevator Scoring Position", 0, true);
        limitSwitch = new DigitalInput(0);
        resetSequence = false; 
    }

    public Command scoreGamepiece() {
        if (scoringPos.get() < 1 || scoringPos.get() > 3) {
            System.out.println("please give an actual scoring location");
            return runOnce(() -> {});   // hope that works
        } else {
            Manipulator manipulator = Manipulator.getInstance();
            return moveElevator(Constants.Elevator.SETPOINTS[(int)scoringPos.get()], Constants.Elevator.MAX_ERROR).
            andThen(manipulator.run()).until(manipulator::coralIn).andThen(manipulator.stop()).
            andThen(moveElevator(Constants.Elevator.L0, Constants.Elevator.MAX_ERROR));
        }
    }

    public boolean isAtBottom() {
        return !limitSwitch.get();
    }

    public Command reZero() {
        return runOnce(() -> {
            resetSequence = true;
            super.leadMotor.set(-0.1);
            System.out.println("Reset Sequency Starting");
        }).andThen(run(() -> {
        })).until(() -> isAtBottom()).finallyDo(() -> {
            resetSequence = false;
            System.out.println("Reset Sequency Ending");
            super.leadMotor.set(0.0);
            super.leadMotor.getEncoder().setPosition(0.0);
            super.followMotor.getEncoder().setPosition(0.0);
        });
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("At Bottom", isAtBottom());
        if (!resetSequence) { 
            super.updatePID();
        }
        super.periodic();
        SmartDashboard.putBoolean("At Bottom", isAtBottom());
        if (!resetSequence) { 
            super.updatePID();
        }
    }
}