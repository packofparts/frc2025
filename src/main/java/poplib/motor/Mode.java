package poplib.motor;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public enum Mode {
    COAST(true),
    BRAKE(false);

    private boolean mode;

    private Mode(boolean mode) {
        this.mode = mode;
    }

    public NeutralModeValue getTalonMode() {
        return mode ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    }

    public IdleMode getSparkMaxMode() {
        return mode ? IdleMode.kCoast : IdleMode.kBrake;
    }
};
