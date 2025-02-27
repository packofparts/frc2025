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
        // robotContainer.intake.resetToAbsPosition();
        robotContainer.swerve.updateEncoders(); 
        // robotContainer.swerve.updateEncoders(); 
    }

    @Override
    public void autonomousInit() {
        autoCommand = robotContainer.getAutonomousCommand();

        if (autoCommand != null) {
            System.out.println("Auto running: " + autoCommand.getName());
            autoCommand.schedule();
        }
        else{
            System.out.println("auto is null"); 
        }
    }

    @Override
    public void autonomousPeriodic() { }

    @Override
    public void teleopInit() {
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        if (robotContainer.getIntaking() && robotContainer.manipulator.coralIn()) {
            robotContainer.stopIntaking().schedule();

            // if (robotContainer.scoring.getSelected() == Constants.SCORING_SETPOINTS.L4) {
            //     robotContainer.l4HoldManipulator().schedule();
            // }
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
