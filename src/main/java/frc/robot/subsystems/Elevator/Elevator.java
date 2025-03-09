// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.TalonElevator;

public class Elevator extends TalonElevator {
    private static Elevator instance;
    private final DigitalInput limitSwitch;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    public Elevator() {
        super(
            Constants.Elevator.RIGHT_MOTOR, 
            Constants.Elevator.LEFT_MOTOR, 
            Constants.Elevator.FF_CONFIG, 
            Constants.Elevator.TUNNING_MODE, 
            "Elevator"
        );

        limitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_CHANNEL);
    }

    public boolean isAtBottom() {
        return !limitSwitch.get();
    }

    public Command reZero() {
        return runOnce(() -> System.out.println("Reset Sequency Starting"))
            .andThen(moveDown(Constants.Elevator.RESET_SPEED))
            .until(() -> isAtBottom())
            .finallyDo(() -> {
                System.out.println("Reset Sequency Ending");
                zeroPosition();
            })
            .andThen(stop());
    }

    public Command moveElev(){
        return runOnce(() -> {
            setpoint.setDefault(Constants.SCORING_SETPOINTS.L3.getElevator());
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator at bottom", isAtBottom());
        SmartDashboard.putNumber("ele setpoint", setpoint.get());
        super.periodic();
    }
}
