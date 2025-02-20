// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import poplib.control.FFConfig;
import poplib.motor.FollowerConfig;
import poplib.motor.MotorConfig;
import poplib.sensors.absolute_encoder.AbsoluteEncoderConfig;
import poplib.smart_dashboard.PIDTuning;
import poplib.subsytems.pivot.Pivot;

public class TalonPivot extends Pivot {
    public final TalonFX leadMotor;
    @SuppressWarnings("unused")
    protected TalonFX followerMotor;
    protected PIDTuning pid;
    protected PositionDutyCycle position;

    public TalonPivot(MotorConfig leadConfig, FollowerConfig followerConfig, double gearRatio, FFConfig ffConfig, AbsoluteEncoderConfig absoluteConfig, boolean tuningMode, String subsytemName) {
        super(ffConfig, absoluteConfig, tuningMode, subsytemName);
        leadMotor = leadConfig.createTalon();
        if (followerConfig != null) {
            followerMotor = followerConfig.createTalon();
        } else {
            followerMotor = null;
        };

        pid = leadConfig.genPIDTuning("Pivot Motor " + subsytemName, tuningMode);
        position = new PositionDutyCycle(0.0);
        position.withSlot(leadMotor.getClosedLoopSlot().getValue());

        resetToAbsolutePosition();
    }

    public boolean atSetpoint(double error, double setpoint) {
        return getError(setpoint) < error;
    }

    public double getError(double setpoint) {
        return Math.abs(leadMotor.getPosition().getValueAsDouble() - setpoint);
    }

    public void updatePID() {
        leadMotor.setControl(position.withPosition(super.setpoint.get()).withFeedForward(super.ff.getKg()));
    }

    @Override
    public void log() {
        super.log();
        SmartDashboard.putNumber("Lead Position " + getName(), leadMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        pid.updatePID(leadMotor);
        leadMotor.setControl(position.withPosition(super.setpoint.get()).withFeedForward(super.ff.getKg()));
    }

    @Override
    public void resetToAbsolutePosition() {
        leadMotor.setPosition(getAbsolutePosition());
    }
}