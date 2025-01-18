package poplib.subsytems.pivot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import poplib.control.FFConfig;
import poplib.motor.FollowerConfig;
import poplib.motor.MotorConfig;
import poplib.smart_dashboard.PIDTuning;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkPivot extends Pivot {
    private final SparkMax leadMotor;
    @SuppressWarnings("unused")
    private SparkMax followerMotor;
    private final PIDTuning pid;

    public SparkPivot(MotorConfig leadConfig, FFConfig ffConfig, boolean tuningMode, String subsytemName) {
        super(ffConfig, tuningMode, subsytemName);
        leadMotor = leadConfig.createSparkMax();
        followerMotor = null;
        pid = leadConfig.genPIDTuning("Pivot Motor " + subsytemName, tuningMode);
        leadMotor.getEncoder().setPosition(0.0);
    }

    public SparkPivot(MotorConfig leadConfig, FollowerConfig followerConfig, FFConfig ffConfig, boolean tuningMode, String subsytemName) {
        this(leadConfig, ffConfig, tuningMode, subsytemName);
        followerMotor = followerConfig.createSparkMax(leadMotor);
    }

    public boolean atSetpoint(double error, double setpoint) {
        return getError(setpoint) < error;
    }

    public double getError(double setpoint) {
        return Math.abs(leadMotor.getEncoder().getPosition() - setpoint);
    }

    @Override
    public void log() {
        super.log();
        SmartDashboard.putNumber("Lead Position " + getName(), leadMotor.getEncoder().getPosition());
    }

    @Override
    public void periodic() {
        pid.updatePID(leadMotor);

        // TODO: Move to mutable units
        leadMotor.getClosedLoopController().setReference(
            setpoint.get(), 
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ff.calculate(leadMotor.getEncoder().getPosition(), 0)
        );
    }

    @Override
    public void resetToAbsolutePosition() {
        leadMotor.getEncoder().setPosition(getAbsolutePosition());
    } 
}
