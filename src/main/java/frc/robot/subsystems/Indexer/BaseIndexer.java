// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import poplib.motor.MotorConfig;

public class BaseIndexer extends SubsystemBase {
  private SparkMax motor1;
  private SparkMax motor2;
  private double indexerSpeed;
  
  /** Creates a new BaseIndexer. */
  public BaseIndexer(MotorConfig motor1Config, MotorConfig motor2Config, double speed) {
    motor1 = motor1Config.createSparkMax();
    if(motor2Config != null){
      motor2 = motor2Config.createSparkMax();
    }
    else{
      motor2 = null;
    }
    indexerSpeed = speed;
  }

  public Command run() {
        return runOnce(() -> {
            motor1.set(indexerSpeed);
            if (motor2 != null){
              motor2.set(indexerSpeed);
            }
        });
    }

  public Command stop() {
      return runOnce(() -> {
        motor1.set(0.0);
        if (motor2 != null){
          motor2.set(0.0);
        }
      });
  }

  public Command reverse() {
      return runOnce(() -> {
        motor1.set(-1 * indexerSpeed);
        if (motor2 != null){
          motor2.set(-1 * indexerSpeed);
        }
      });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
