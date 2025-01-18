package poplib.subsytems.flywheel;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import poplib.motor.MotorConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonFlywheel extends Flywheel {
    public TalonFX leadMotor; 
    public TalonFX followerMotor; 

    private VelocityDutyCycle velocity;
    private CoastOut idleControl;
    
    protected TalonFlywheel(MotorConfig leadConfig, MotorConfig followerConfig, String subsytemName, boolean tuningMode, boolean motorsInverted) {
        super(subsytemName, tuningMode);

        this.leadMotor = leadConfig.createTalon();
        this.followerMotor = followerConfig.createTalon();

        this.velocity = new VelocityDutyCycle(0.0);
        this.idleControl = new CoastOut();

        followerMotor.setControl(new Follower(leadConfig.canId, motorsInverted));
    } 

    public double getError(double setpoint) {
        return Math.abs(leadMotor.getVelocity().getValueAsDouble() - setpoint);
    }

    public void log() {
        SmartDashboard.putNumber(getName() + " velocity ", leadMotor.getVelocity().getValueAsDouble());
    }

    public double getVelocity() {
        return leadMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        leadPidTuning.updatePID(leadMotor);

        if (setpoint.hasChanged()) {
            leadMotor.setControl(setpoint.get() != 0 ? velocity.withVelocity(setpoint.get()) : idleControl);
        }
     } 
}
