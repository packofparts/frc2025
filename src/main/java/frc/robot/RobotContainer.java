// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Manipulator.Manipulator;
import poplib.controllers.oi.OI;
import poplib.controllers.oi.XboxOI;
import poplib.swerve.commands.SysIdSwerve;
import poplib.swerve.commands.TeleopSwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
    public final Swerve swerve;
    public final Elevator elevator;
    public final Indexer indexer;
    public final Intake intake;
    public final Manipulator manipulator;

    private final SysIdSwerve sys;

    public final OI oi;

    private final SendableChooser<Command> autoChooser;
    public final SendableChooser<Command> scoring;

    private boolean intaking;

    public RobotContainer() {
        // Subsytem intatiating
        swerve = Swerve.getInstance();
        elevator = Elevator.getInstance();
        manipulator = Manipulator.getInstance();
        indexer = Indexer.getInstance();
        intake = Intake.getInstance();

        sys = new SysIdSwerve(swerve);
        oi = XboxOI.getInstance();


        // Scoring Selecter
        scoring = new SendableChooser<>();
        scoring.addOption("L1", elevatorScore(Constants.Elevator.SETPOINTS.L1));
        scoring.addOption("L2", elevatorScore(Constants.Elevator.SETPOINTS.L2));
        scoring.addOption("L3", elevatorScore(Constants.Elevator.SETPOINTS.L3));
        scoring.addOption("L4", elevatorScore(Constants.Elevator.SETPOINTS.L4));
        SmartDashboard.putData(scoring);

        // Auto Selecter
        autoChooser = AutoBuilder.buildAutoChooser();    
        autoChooser.addOption("none", new InstantCommand(() -> {}));
        autoChooser.addOption("One Score Close", new PathPlannerAuto("One Score Close"));
        autoChooser.addOption("One Score Far", new PathPlannerAuto("One Score Far"));
        autoChooser.addOption("line_2meters", new PathPlannerAuto("line_2meters"));
        autoChooser.addOption("square", new PathPlannerAuto("square"));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerve.setDefaultCommand(new TeleopSwerveDrive(swerve, oi));

        intaking = false;
        configureBindings();
    }

    public boolean getIntaking() {
        return intaking;
    }


    private void configureBindings() {
        // oi.getDriverButton(XboxController.Button.kA.value).whileTrue(
        //     new WheelRadiusChar(swerve, Constants.Swerve.MODULE_TYPE, Constants.Swerve.DRIVE_BASE_RADIUS)
        // );
        // oi.getDriverButton(XboxController.Button.kB.value).whileTrue(sys.sysIdDynamic(Direction.kForward));

        // oi.getDriverButton(XboxController.Button.kX.value).whileTrue(sys.sysIdQuasistatic(Direction.kForward));
        // oi.getDriverButton(XboxController.Button.kY.value).whileTrue(sys.sysIdDynamic(Direction.kForward));

        oi.getDriverButton(XboxController.Button.kY.value).onTrue(togleIntake());

        // oi.getDriverButton(XboxController.Button.kB.value).onTrue(elevator.moveUp(0.4)).onFalse(elevator.stop());
        // oi.getDriverButton(XboxController.Button.kX.value).onTrue(elevator.moveDown(0.4)).onFalse(elevator.stop());

        // oi.getDriverButton(XboxController.Button.kA.value).onTrue(manipulator.reverse());

        oi.getDriverButton(XboxController.Button.kA.value).onTrue(new InstantCommand(() -> {
            scoring.getSelected().schedule();
        }));
    }

    public Command togleIntake() {
        return new InstantCommand(() -> {
            (!intaking ? startIntaking() : stopIntaking()).schedule();
            intaking = !intaking;
        });
    }

    public Command startIntaking() {
       return new SequentialCommandGroup(
            intake.moveWrist(Constants.Intake.SETPOINTS.CORAL_PICKUP.getSetpoint(), Constants.Intake.MAX_ERROR),
            new ParallelCommandGroup(
                manipulator.run(),
                indexer.run(),
                intake.run()
            )
        );
    }

    public Command stopIntaking() {
       return intake.moveWrist(Constants.Intake.SETPOINTS.IDLE.getSetpoint(),Constants.Intake.MAX_ERROR).
            andThen(
                new ParallelCommandGroup(
                    manipulator.stop(),
                    indexer.stop(),
                    intake.stop()
                )
        );
    }

    public Command elevatorScore(Constants.Elevator.SETPOINTS setpoint) {
        return new InstantCommand(() -> System.out.println("Setpoint Moving: " + setpoint.getSetpoint())).
            andThen(elevator.moveElevator(setpoint.getSetpoint(), Constants.Elevator.MAX_ERROR).
            andThen(manipulator.run()).
            andThen(new WaitCommand(1.0)).
            andThen(manipulator.stop()).
            andThen(elevator.moveElevator(Constants.Elevator.SETPOINTS.IDLE.getSetpoint(), Constants.Elevator.MAX_ERROR))
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
