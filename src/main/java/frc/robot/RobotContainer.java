// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;
import poplib.controllers.oi.OI;
import poplib.controllers.oi.XboxOI;
import poplib.smart_dashboard.TunableNumber;
import poplib.swerve.commands.TeleopSwerveDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    Swerve swerve;
    Elevator elevator;
    Indexer indexer;
    Intake intake;
    Manipulator manipulator;
    OI oi;
    SendableChooser<Command> scoring;

    public RobotContainer() {
        swerve = Swerve.getInstance();
        oi = XboxOI.getInstance();
        elevator = frc.robot.subsystems.Elevator.getInstance();
        manipulator = Manipulator.getInstance();
        indexer = Indexer.getInstance();
        intake = Intake.getInstance();

        scoring = new SendableChooser<Command>();
        scoring.addOption("L1", elevatorScore(Constants.Elevator.SETPOINTS.L1));
        scoring.addOption("L2", elevatorScore(Constants.Elevator.SETPOINTS.L2));
        scoring.addOption("L3", elevatorScore(Constants.Elevator.SETPOINTS.L3));
        SmartDashboard.putData(scoring);

        swerve.setDefaultCommand(new TeleopSwerveDrive(swerve, oi));
        configureBindings();
    }


    private void configureBindings() {
        // SysIdSwerve sys = new SysIdSwerve(swerve);
        // SysIdSwerve sys = new SysIdSwerve(swerve);

        // oi.getDriverButton(XboxController.Button.kA.value).whileTrue(sys.sysIdQuasistatic(Direction.kForward));
        // oi.getDriverButton(XboxController.Button.kB.value).whileTrue(sys.sysIdDynamic(Direction.kForward));

        oi.getDriverButton(XboxController.Button.kY.value).onTrue(elevator.reZero());

        // Intake 
        oi.getDriverButton(XboxController.Button.kB.value).onTrue(
        new SequentialCommandGroup(
            intake.moveWrist(Constants.Intake.SETPOINTS.CORAL_PICKUP.getSetpoint(), 1),
            new ParallelCommandGroup(
                manipulator.run(),
                indexer.run(),
                intake.run()
            )
        )).onFalse(
            new SequentialCommandGroup(
                intake.moveWrist(Constants.Intake.SETPOINTS.IDLE.getSetpoint(), 1),
            new ParallelCommandGroup(
            manipulator.stop(),
            indexer.stop(),
            intake.stop()
        )));

        oi.getDriverButton(XboxController.Button.kX.value).onTrue(new ParallelCommandGroup(
            intake.reverse()
        )).onFalse(new ParallelCommandGroup(
            intake.stop()
        ));

        oi.getDriverButton(XboxController.Button.kA.value).onTrue(new InstantCommand(() -> { 
            ((Command) scoring.getSelected()).schedule();
        }));

        // oi.getDriverButton(XboxController.Button.kX.value).onTrue(elevator.moveDown(0.5)).onFalse(elevator.stop());

    }

    public Command intakeCoral() {
        Elevator elevator = Elevator.getInstance();
        Indexer indexer = Indexer.getInstance();
        Manipulator manipulator = Manipulator.getInstance();
        return elevator.moveElevator(Constants.Elevator.SETPOINTS.IDLE.getSetpoint(), Constants.Elevator.MAX_ERROR).
        alongWith(intake.moveWrist(Constants.Intake.SETPOINTS.CORAL_PICKUP.getSetpoint(), Constants.Intake.MAX_ERROR)).
        andThen(intake.run()).
        alongWith(indexer.run()).
        alongWith(manipulator.run()).
        until(manipulator::coralIn).
        andThen(manipulator.stop()).
        alongWith(indexer.stop()).
        alongWith(intake.stop()).
        andThen(intake.moveWrist(Constants.Intake.SETPOINTS.IDLE.getSetpoint(), Constants.Intake.MAX_ERROR));
    }

    public Command elevatorScore(Constants.Elevator.SETPOINTS setpoint) {
        return 
            new InstantCommand(() -> {
                System.out.println("Setpoint Movinf: " + setpoint.getSetpoint());
            }).andThen(elevator.moveElevator(setpoint.getSetpoint(), Constants.Elevator.MAX_ERROR).
            andThen(manipulator.run()).andThen(new WaitCommand(1.0)).andThen(manipulator.stop())
            .andThen(elevator.moveElevator(Constants.Elevator.SETPOINTS.IDLE.getSetpoint(), Constants.Elevator.MAX_ERROR)));
    }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
    return null;
    }
}
