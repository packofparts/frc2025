// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import poplib.math.MathUtil;
import poplib.smart_dashboard.PIDTuning;
import poplib.smart_dashboard.TunableNumber;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final SparkMax rightMotor;
    private final SparkMax leftMotor;
    private final DigitalInput limitSwitch;

    private final TunableNumber setpoint;
    private final PIDTuning tuning;

    private boolean resetSequence;

    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private Elevator() {        
        rightMotor = Constants.Elevator.RIGHT_MOTOR.createSparkMax();
        leftMotor = Constants.Elevator.LEFT_MOTOR.createSparkMax(rightMotor);
        tuning = Constants.Elevator.RIGHT_MOTOR.genPIDTuning("Elevator Right Motor", Constants.Elevator.TUNNING_MODE);

        rightMotor.getEncoder().setPosition(0.0);
        leftMotor.getEncoder().setPosition(0.0);

        limitSwitch = new DigitalInput(0);

        resetSequence = false; 

        setpoint = new TunableNumber("Elevator Setpoint", 0, Constants.Elevator.TUNNING_MODE); 
    }

    public Command moveElevator(double setPoint) {
        return run(() -> {
            setpoint.setDefault(setPoint);
        }).until(() ->  MathUtil.getError(rightMotor, setPoint) < Constants.Elevator.MAX_ERROR);
    }

    public Command moveUp() {
        return runOnce(() -> {
            rightMotor.set(Math.abs(Constants.Elevator.MOTOR_SPEED));
        });
    }

    public Command moveDown() {
        return runOnce(() -> {
            rightMotor.set(-Math.abs(Constants.Elevator.MOTOR_SPEED));
        });
    }

    public Command stop() {
        return runOnce(() -> {
            rightMotor.set(0.0);
        });
    }

    public boolean isAtBottom() {
        return !limitSwitch.get();
    }

    public Command reZero() {
        return runOnce(() -> {
            resetSequence = true;
            rightMotor.set(-0.1);
            System.out.println("Reset Sequency Starting");
        }).andThen(run(() -> {
        })).until(() -> isAtBottom()).finallyDo(() -> {
            resetSequence = false;
            System.out.println("Reset Sequency Ending");
            rightMotor.set(0.0);
            rightMotor.getEncoder().setPosition(0.0);
            leftMotor.getEncoder().setPosition(0.0);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position Meters", rightMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Left Elevator Position", leftMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("At Bottom", isAtBottom());

        if (!resetSequence) { 
            rightMotor.getClosedLoopController().setReference(
                setpoint.get(), 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                Constants.Elevator.FF.calculate(0.0)
            );
        }

        tuning.updatePID(rightMotor);
    }
}
