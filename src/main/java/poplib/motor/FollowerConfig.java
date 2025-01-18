package poplib.motor;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import poplib.error_handling.ErrorHandling;

public class FollowerConfig {
    private MotorConfig leadConfig;
    private boolean inverted;
    private int canId;

    public FollowerConfig(MotorConfig leadConfig, boolean inverted, int canId) {
        this.leadConfig = leadConfig;
        this.inverted = inverted;
        this.canId = canId;
    }

    public SparkMax createSparkMax(SparkMax lead) { 
        SparkMax motor = new SparkMax(canId, SparkMax.MotorType.kBrushless);
        SparkMaxConfig config = leadConfig.getSparkMaxConfig();
        config.follow(leadConfig.canId, inverted);
        
        ErrorHandling.handlRevLibError(
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
            "configuring follower motor " + canId
        );

        return motor;
    }

    public TalonFX createTalon() {
        TalonFX motor = new TalonFX(canId);
        motor.getConfigurator().apply(leadConfig.getTalonConfig());
        motor.setControl(new Follower(leadConfig.canId, inverted));
        return motor;
    }
}
