// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import POPLib.Sensors.BeamBreak.BeamBreak;
import POPLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  private BeamBreak beambrake;
  private SparkMax manipulatorMotor;
  private TunableNumber setpoint; 
  private PIDController pidController;
  private static Manipulator instance;

  public static Manipulator getInstance() {
      if (instance == null) {
          instance = new Manipulator();
      }

      return instance;
  }
  
  private Manipulator() {
    manipulatorMotor = new SparkMax(0, MotorType.kBrushless);
    beambrake = ManipulatorConstants.breakConfig.createBeamBreak();
    pidController = new PIDController(0,0,0);
  }

  public void setSetpoint(double sP){
    setpoint.setDefault(sP);
  }

  public void runManipulator(double setpoint) {
    pidController.setSetpoint(setpoint);
    double output = pidController.calculate(manipulatorMotor.get());
    manipulatorMotor.set(output);
  }
  
  public void stopManipulator(){
    manipulatorMotor.set(0);
  }

  public Command takeIn(double setpoint){
    return runOnce(()->{
      runManipulator(setpoint);
    }).until(beambrake.getBlockedSupplier()).andThen(this::stopManipulator);
  }

  public Command takeOut(double setpoint){
    return runOnce(() -> {
      runManipulator(setpoint);
    }).until(beambrake.getUnBlockedSupplier()).andThen(this::stopManipulator);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
