// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private Elevator() {}


    @Override
    public void periodic() {
    }
}
