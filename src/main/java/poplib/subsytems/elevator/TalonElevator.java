package poplib.subsytems.elevator;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import poplib.control.FFConfig;
import poplib.math.MathUtil;
import poplib.motor.FollowerConfig;
import poplib.motor.MotorConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TalonElevator extends Elevator {
    TalonFX leadMotor;
    TalonFX followMotor;
    boolean usePID;
    PositionDutyCycle position;

    public TalonElevator(MotorConfig motorConfig, FollowerConfig followerConfig, FFConfig ffConfig, boolean tuningMode, boolean usePID, String subsystemName) {
        super(ffConfig, tuningMode, subsystemName);
    
        this.usePID = usePID;

        leadMotor = motorConfig.createTalon();
        position = new PositionDutyCycle(0.0).
        withSlot(leadMotor.getClosedLoopSlot().getValue());

        leadMotor.setPosition(0.0);
        followMotor = followerConfig.createTalon();
        followMotor.setPosition(0.0);
    }

    @Override
    public void periodic() {            
        super.tuning.updatePID(leadMotor);
        SmartDashboard.putNumber("Elevator lead motor pos", getEncoderPos());
        SmartDashboard.putNumber("Elevator follow motor pos", followMotor.getPosition().getValue().in(Units.Rotations));
    }

    public void updatePID() {
        if (usePID) {
            leadMotor.setControl(position.withPosition(super.setpoint.get()).withFeedForward(super.feedforward.getKg()));
        }
    }

    public double getEncoderPos() {
        return leadMotor.getPosition().getValue().in(Units.Rotations);
    }

    public double getError(double setpoint) {
        return MathUtil.getError(leadMotor, setpoint);
    }

    /**
     * Requires usePID to be false in order to work
     */
    public Command moveUp(double speed) {
        return runOnce(() -> {
            leadMotor.set(Math.abs(speed));
        });
    }

    /**
     * Requires usePID to be false in order to work
     */
    public Command moveDown(double speed) {
        return runOnce(() -> {
            leadMotor.set(-Math.abs(speed));
        });
    }
    
    /**
     * Requires usePID to be false in order to work
     */
    public Command stop() {
        return runOnce(() -> {
            leadMotor.set(0.0);
        });
    }    
}
