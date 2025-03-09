package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.TalonPivot;

public class Intake extends TalonPivot {
    private final TalonFX spin;
    private final DutyCycleOut control;
    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    private Intake() {
        super(
            Constants.Intake.PIVOT, 
            null, 
            Constants.Intake.GEAR_RATIO,
            Constants.Intake.FF, 
            Constants.Intake.ENCODER, 
            Constants.Intake.TUNING_MODE, 
            "Intake"
        );

        spin = Constants.Intake.SPIN.createTalon();
        control = new DutyCycleOut(0.0);

        this.leadMotor.setPosition(0.0);
        // deleted reset to abs enc
        setpoint.setDefault(Constants.Intake.SETPOINTS.IDLE.getSetpoint());
    }

    public Command run() {
        return runOnce(() -> {
            spin.setControl(control.withOutput(Constants.Intake.SPEED));
        }); 
    }

    public Command stop() {
        return runOnce(() -> {
            spin.setControl(control.withOutput(0.0));
        }); 
    }

    public Command reverse() {
        return runOnce(() -> {
            spin.setControl(control.withOutput(-1 * Constants.Intake.SPEED));
        }); 
    }

    @Override
    public void periodic() {
        super.periodic();
        super.log();
    }
}