package frc.robot.util;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import poplib.control.FFConfig;
import poplib.math.MathUtil;
import poplib.motor.FollowerConfig;
import poplib.motor.MotorConfig;
import poplib.smart_dashboard.TunableNumber;
import poplib.subsytems.elevator.Elevator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TalonElevator extends Elevator {
    private final TalonFX leadMotor;
    private final TalonFX followMotor;
    private final PositionDutyCycle position;
    private final TunableNumber kG;
    protected boolean usePID;

    public TalonElevator(MotorConfig motorConfig, FollowerConfig followerConfig, FFConfig ffConfig, boolean tuningMode, String subsystemName) {
        super(
            ffConfig, 
            tuningMode, 
            subsystemName
        );
    
        leadMotor = motorConfig.createTalon();
        leadMotor.setPosition(0.0);

        followMotor = followerConfig.createTalon();
        followMotor.setPosition(0.0);

        position = new PositionDutyCycle(0.0).
        withSlot(leadMotor.getClosedLoopSlot().getValue());

        kG = new TunableNumber(subsystemName + " kG", ffConfig.G, tuningMode);

        usePID = true;
    }

    @Override
    public void periodic() {            
        super.tuning.updatePID(leadMotor);

        if (usePID) {
            leadMotor.setControl(position.withPosition(super.setpoint.get()).withFeedForward(kG.getDefault()));
        }

        SmartDashboard.putNumber("Elevator lead motor pos", getEncoderPos());
        SmartDashboard.putNumber("Elevator follow motor pos", followMotor.getPosition().getValue().in(Units.Rotations));
    }


    public double getEncoderPos() {
        return leadMotor.getPosition().getValue().in(Units.Rotations);
    }

    public double getError(double setpoint) {
        return MathUtil.getError(leadMotor, setpoint);
    }


    public Command moveUp(double speed) {
        return runOnce(() -> {
            usePID = false;
            leadMotor.set(Math.abs(speed));
        });
    }


    public Command moveDown(double speed) {
        return runOnce(() -> {
            usePID = false;
            leadMotor.set(-Math.abs(speed));
        });
    }
    
    public Command stop() {
        return runOnce(() -> {
            usePID = true;
            leadMotor.set(0.0);
        });
    }

    public void zeroPosition() {
        leadMotor.setPosition(0.0);
        followMotor.setPosition(0.0);
    }
}
