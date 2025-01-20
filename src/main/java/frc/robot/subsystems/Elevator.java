// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import poplib.subsytems.elevator.SparkElevator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


public class Elevator extends SparkElevator {
    private final DigitalInput limitSwitch;
    private boolean resetSequence;
    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private Elevator() { 
        super(Constants.Elevator.RIGHT_MOTOR, Constants.Elevator.LEFT_MOTOR, 
        Constants.Elevator.FF_CONFIG, Constants.Elevator.TUNNING_MODE, true, "elevator");   
        
        limitSwitch = new DigitalInput(0);
        resetSequence = false; 
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