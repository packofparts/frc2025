package poplib.motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;

public class ConversionConfig {
     public double gearRatio;
     public AngleUnit unit; 

     public ConversionConfig(double gearRatio, AngleUnit unit) {
          this.gearRatio = gearRatio;
          this.unit = unit;
     }

     public ConversionConfig() {
          this(1.0, Units.Rotations);
     }

     public void updateConfig(SparkMaxConfig config) {
          // TODO: VERIFY THIS WORKS
          config.encoder.positionConversionFactor(unit.convertFrom(1.0, Units.Rotation) / gearRatio);
          config.encoder.velocityConversionFactor((unit.convertFrom(1.0, Units.Rotation) / gearRatio) / 60);
     }

     public void updateConfig(TalonFXConfiguration config) {
          // TODO: VERIFY THIS WORKS
          System.out.println(unit.convertFrom(1.0, Units.Rotation));
          config.Feedback.SensorToMechanismRatio =  1.0 / (unit.convertFrom(1.0, Units.Rotation) / gearRatio);
     }
}
