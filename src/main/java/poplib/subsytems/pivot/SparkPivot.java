package poplib.subsytems.pivot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import poplib.control.FFConfig;
import poplib.motor.FollowerConfig;
import poplib.motor.MotorConfig;
import poplib.sensors.absolute_encoder.AbsoluteEncoderConfig;
import poplib.smart_dashboard.PIDTuning;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkPivot extends Pivot {
    public final SparkMax leadMotor;
    @SuppressWarnings("unused")
    private final SparkMax followerMotor;
    private final PIDTuning pid;

    public SparkPivot(MotorConfig leadConfig, FollowerConfig followerConfig, double gearRatio, FFConfig ffConfig, AbsoluteEncoderConfig absoluteConfig, boolean tuningMode, String subsytemName) {
        super(ffConfig, absoluteConfig, tuningMode, subsytemName);
        leadMotor = leadConfig.createSparkMax();
        if (followerConfig != null) {
            followerMotor = followerConfig.createSparkMax();
        } else {
            followerMotor = null;
        };

        pid = leadConfig.genPIDTuning("Pivot Motor " + subsytemName, tuningMode);

        resetToAbsolutePosition();
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
