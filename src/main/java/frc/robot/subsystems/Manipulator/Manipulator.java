// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;


import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
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

        range = new CANrange(Constants.Manipulator.RANGE_ID, Constants.Ports.CANIVORE_NAME);
        CANrangeConfiguration config = new CANrangeConfiguration();
        config.ProximityParams.ProximityThreshold = 0.1;
        range.getConfigurator().apply(config);

        setpoint.setDefault(Constants.SCORING_SETPOINTS.IDLE.getManipulator());
    }

    public Command run(){
        return run(Constants.Manipulator.SPEEDS.NORMAL);
    }

    public Command run(Constants.Manipulator.SPEEDS speed){
        return runOnce(() -> {
            spin.setControl(control.withOutput(speed.getSpeed()));
        }); 
    }

    public Command stop(){
        return runOnce(() ->{
            spin.setControl(control.withOutput(0.0));
        });
    }

    public Command reverse() {
        return reverse(Constants.Manipulator.SPEEDS.NORMAL);
    }

    public Command reverse(Constants.Manipulator.SPEEDS speed) {
        return runOnce(() ->{
            spin.setControl(control.withOutput(-1 * speed.getSpeed()));
        });
    }

    public boolean coralIn() {
        return range.getIsDetected().getValue();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Blocked", coralIn());
        super.log();
        super.periodic();
    }
}
