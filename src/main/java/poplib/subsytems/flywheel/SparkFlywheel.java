package poplib.subsytems.flywheel;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import poplib.math.MathUtil;
import poplib.motor.FollowerConfig;
import poplib.motor.MotorConfig;
import poplib.smart_dashboard.PIDTuning;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkFlywheel extends Flywheel {
    SparkMax leadMotor; 
    SparkMax followerMotor; 
    PIDTuning leadPidTuning;
 
    protected SparkFlywheel(MotorConfig leadConfig, FollowerConfig followerConfig, String subsytemName, boolean tuningMode) {
        super(subsytemName, tuningMode);

        this.leadMotor = leadConfig.createSparkMax();
        this.followerMotor = followerConfig.createSparkMax();
    } 

    public double getError(double setpoint) {
        return MathUtil.getError(leadMotor, setpoint);
    }

    public void log() {
        SmartDashboard.putNumber(getName() + " velocity ", leadMotor.getEncoder().getVelocity());
    }

    @Override
    public void periodic() {
        leadPidTuning.updatePID(leadMotor);
        leadMotor.getClosedLoopController().setReference(setpoint.get(), ControlType.kVelocity);
    }
}
