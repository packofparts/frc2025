// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import POPLib.Motor.MotorConfig;
import POPLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private SparkMax rightMotor;
    private SparkMax leftMotor;
    private TunableNumber setpoint;

    private Elevator() {        
        rightMotor = Constants.Elevator.rightMotorConfig.createSparkMax();
        leftMotor = Constants.Elevator.leftMotorConfig.createSparkMax(rightMotor);
        setpoint = new TunableNumber("Elevator Setpoint", 0, false); 
    }
    
    public Command moveElevator(double setPoint) {
        // PIDController pid = Constants.Elevator.rightMotorConfig.pid.getPIDController();
        // pid.setSetpoint(Constants.Elevator.upperSetpoint);

        return run(() -> {
            rightMotor.getClosedLoopController().setReference(setPoint, null);
        }).until(() -> {
            return Math.abs(rightMotor.getEncoder().getPosition() - setPoint) < 0.2;
        });
    }

    public Command moveUp() {
        return runOnce(() -> {
            rightMotor.set(0.3);
        });
    }

    public Command moveDown() {
        return runOnce(() -> {
            rightMotor.set(-0.3);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            rightMotor.set(0.0);
        });
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevatorPos", rightMotor.getEncoder().getPosition());
    }
}
