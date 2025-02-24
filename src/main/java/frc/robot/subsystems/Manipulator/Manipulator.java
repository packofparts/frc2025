// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;

import java.net.ContentHandler;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.TalonPivot;

public class Manipulator extends TalonPivot {
    private final TalonFX spin;
    private final DutyCycleOut control;
    private static CANrange range;
    
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
        spin = Constants.Manipulator.MANIPULATOR_MOTOR.createTalon();
        control = new DutyCycleOut(0.0);
    }

    public Command run(){
        return runOnce(() ->{
            spin.setControl(control.withOutput(Constants.Manipulator.SPEED));
        });
    }

    public Command stop(){
        return runOnce(() ->{
            spin.setControl(control.withOutput(0.0));
        });
    }

    public Command reverse(){
        return runOnce(() ->{
            spin.setControl(control.withOutput(-1 * Constants.Manipulator.SPEED));
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Blocked", range.getIsDetected().getValue());
        super.periodic();
    }
}
