// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;
import java.util.Optional;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.SCORING_SETPOINTS;
import frc.robot.util.RobotZoneDetector;

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
        
        FollowPathCommand.warmupCommand().schedule();
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
        // if (robotContainer.getIntaking() && robotContainer.manipulator.coralIn()) {
        //     if (robotContainer.scoring.getSelected() == Constants.SCORING_SETPOINTS.L4) {
        //         robotContainer.turnOfIntaking();
        //         (robotContainer.stopIntaking().andThen(robotContainer.l4HoldManipulator())).schedule();
        //     } else {
        //         // robotContainer.stopIntaking().schedule();
        //     }
        // }

        if(robotContainer.getIntaking() && robotContainer.manipulator.coralIn()){
            robotContainer.stopIntaking().schedule();
            if(robotContainer.scoring.getSelected() == Constants.SCORING_SETPOINTS.L4){
                robotContainer.l4HoldManipulator().schedule();;
            }
        }


        SmartDashboard.putBoolean("intaking", robotContainer.getIntaking());

        // alliance color update
        Pose2d currPose = robotContainer.swerve.getOdomPose();
        Optional<Alliance> thing = DriverStation.getAlliance();
        if (thing.isPresent()) {
            Constants.AutoAlign.IS_BLUE = thing.get() == DriverStation.Alliance.Blue;
        }
        SmartDashboard.putBoolean("isBlue", Constants.AutoAlign.IS_BLUE);

        // robot zone update
        Constants.AutoAlign.ROBOT_ZONE =
        RobotZoneDetector.getZone(currPose.getX(), currPose.getY(), Constants.AutoAlign.IS_BLUE);
        SmartDashboard.putNumber("zone", Constants.AutoAlign.ROBOT_ZONE);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
        robotContainer.elevatorScore(SCORING_SETPOINTS.L4);
        CommandScheduler.getInstance().schedule(robotContainer.goToScoringPosition(SCORING_SETPOINTS.L4));
        

    }
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
