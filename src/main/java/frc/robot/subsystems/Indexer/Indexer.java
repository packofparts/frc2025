package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private static Indexer instance;
    private final TalonFX motor;
    private final DutyCycleOut control;

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }

        return instance;
    }

    private Indexer() {
        motor = Constants.Indexer.MOTOR.createTalon();
        control = new DutyCycleOut(0);
    }

    public Command run() {
        return runOnce(() -> {
            motor.setControl(control.withOutput(Constants.Indexer.SPEED));
        }); 
    }

    public Command stop() {
        return runOnce(() -> {
            motor.setControl(control.withOutput(0.0));
        }); 
    }

    public Command reverse() {
        return runOnce(() -> {
            motor.setControl(control.withOutput(-1 * Constants.Indexer.SPEED));
        }); 
    }

    @Override
    public void periodic() {}
}