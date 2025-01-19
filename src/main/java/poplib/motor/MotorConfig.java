package poplib.motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import poplib.control.PIDConfig;
import poplib.smart_dashboard.PIDTuning;


public class MotorConfig {
     public final PIDConfig pid;
     public final int canId;
     public final String canBus;
     public final int currentLimit;
     public final boolean inversion;
     public final Mode mode;
     public ConversionConfig conversion;

     private final static String DEFUALT_CANBUS = "rio";

     public MotorConfig(int canId, String canBus, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode, ConversionConfig conversion) {
          this.canId = canId;
          this.canBus = canBus;
          this.currentLimit = currentLimit;
          this.inversion = inversion;
          this.pid = pid;
          this.mode = mode;
          this.conversion = conversion;
     }

     public MotorConfig(int canId) { this(canId, MotorConfig.DEFUALT_CANBUS, -1, false, PIDConfig.getZeroPid(), Mode.COAST, new ConversionConfig());}

     public MotorConfig(int canId, String canBus) { this(canId, canBus, -1, false, PIDConfig.getZeroPid(), Mode.COAST, new ConversionConfig());}

     public MotorConfig(int canId, String canBus, int currentLimit, Boolean inversion, Mode mode) { this(canId, canBus, currentLimit, inversion, PIDConfig.getZeroPid(), mode, new ConversionConfig());}

     public MotorConfig(int canId, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode) { this(canId, MotorConfig.DEFUALT_CANBUS, currentLimit, inversion, pid, mode, new ConversionConfig()); }

     public MotorConfig(int canId, int currentLimit, Boolean inversion, Mode mode) { this(canId, MotorConfig.DEFUALT_CANBUS, currentLimit, inversion, PIDConfig.getZeroPid(), mode, new ConversionConfig());}

     // No CAN Id Motor, used for configs that apply to multiple motors 
     public MotorConfig(String canBus, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode) { this(-1, canBus, currentLimit, inversion, pid, mode, new ConversionConfig()); }
     public MotorConfig(int currentLimit, Boolean inversion, PIDConfig pid, Mode mode) { this(-1, MotorConfig.DEFUALT_CANBUS, currentLimit, inversion, pid, mode, new ConversionConfig()); }

     public PIDTuning genPIDTuning(String motorName, boolean tuningMode) {
          return new PIDTuning(motorName, pid, tuningMode);
     }

     public TalonFXConfiguration getTalonConfig() {
          TalonFXConfiguration talonConfig = new TalonFXConfiguration();

          MotorHelper.updateStatorCurrentLimit(currentLimit, talonConfig.CurrentLimits);

          talonConfig.MotorOutput.NeutralMode = mode.getTalonMode();
          talonConfig.MotorOutput.Inverted = inversion ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

          pid.updatePidConfig(talonConfig);
          conversion.updateConfig(talonConfig);

          return talonConfig;
     }

     public TalonFX createTalon() {
          TalonFX motor = new TalonFX(canId, canBus);
          motor.getConfigurator().apply(getTalonConfig());
          return motor;
     }

     public SparkMax setConfig(SparkMax motor) {
          MotorHelper.applySparkMaxConfig(getSparkMaxConfig(), motor, ResetMode.kResetSafeParameters);

          return motor;
     }

     public TalonFX setConfig(TalonFX motor) {
          motor.getConfigurator().apply(getTalonConfig());

          return motor;
     }

     public SparkMax createSparkMax() { return createSparkMax(SparkLowLevel.MotorType.kBrushless); }

     public SparkMaxConfig getSparkMaxConfig() {
          SparkMaxConfig config = new SparkMaxConfig();

          config.inverted(inversion);
          config.idleMode(mode.getSparkMaxMode());
          config.smartCurrentLimit(currentLimit);
          pid.setPid(config);
          conversion.updateConfig(config);

          return config;
     }

     public MotorConfig getInvertedConfig() { return new MotorConfig(canId, canBus, currentLimit, !inversion, pid, mode, conversion); }

     public SparkMax createSparkMax(SparkLowLevel.MotorType type) {
          SparkMax motor = new SparkMax(canId, type);
          setConfig(motor);
          return motor;
     }
}
