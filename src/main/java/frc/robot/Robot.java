// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autoCommand;

    private final RobotContainer robotContainer;

    public Robot() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() { 
        robotContainer.intake.resetToAbsolutePosition();
        robotContainer.swerve.updateEncoders(); 
    }

    @Override
    public void autonomousInit() {
        autoCommand = robotContainer.getAutonomousCommand();

        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        if (robotContainer.intaking && robotContainer.manipulator.coralIn()) {
            robotContainer.stopIntaking().schedule();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
