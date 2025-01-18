package poplib.motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import poplib.error_handling.ErrorHandling;
import edu.wpi.first.math.controller.PIDController;

/**
 * Create Motor Functions deprecated as of Novemeber 2023, use MotorConfig instead
 */
public class MotorHelper {
    @Deprecated
    public static void setConversionFactor(SparkMaxConfig config, double factor) {
        config.encoder.positionConversionFactor(factor);
        config.encoder.velocityConversionFactor(factor / 60);
    }

    @Deprecated
    public static void setDegreeConversionFactor(SparkMaxConfig config, double gearing) {
        setConversionFactor(config, 360 / gearing);
    }

    public static TalonFXConfiguration setConversionFactor(TalonFXConfiguration config, double factor) {
        config.Feedback.SensorToMechanismRatio = 1.0 / factor;
        return config;
    }

    public static TalonFXConfiguration setDegreeConversionFactor(TalonFXConfiguration config, double gearing) {
        return setConversionFactor(config, (360.0 / gearing));
    }

    public static CurrentLimitsConfigs createSupplyCurrentLimit(int currentLimit) {
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        updateSupplyCurrentLimit(currentLimit, config);
        return config;
    }

    public static void updateSupplyCurrentLimit(int currentLimit, CurrentLimitsConfigs config) {
        config.SupplyCurrentLimit = currentLimit;
        config.SupplyCurrentLowerLimit = currentLimit;
        config.SupplyCurrentLowerTime = 0;
        config.StatorCurrentLimitEnable = false;
        config.SupplyCurrentLimitEnable = true;
    }

    public static CurrentLimitsConfigs createStatorCurrentLimit(int currentLimit) {
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        updateStatorCurrentLimit(currentLimit, config);
        return config;
    }

    public static void updateStatorCurrentLimit(int currentLimit, CurrentLimitsConfigs config) {
        config.StatorCurrentLimit = currentLimit;
        config.SupplyCurrentLimitEnable = false;
        config.StatorCurrentLimitEnable = true;
    }

    public static PIDController getWpiPidController(double p, double i, double d) {
        return new PIDController(p, i, d);
    }

    public static void applySparkMaxConfig(SparkMaxConfig config, SparkMax motor, ResetMode mode) {
        ErrorHandling.handlRevLibError(
               motor.configure(config, mode, PersistMode.kPersistParameters),
               "configuring motor " + motor.getDeviceId()
        );
    }
}
