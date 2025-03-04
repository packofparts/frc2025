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

import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
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
    public final SendableChooser<Constants.SCORING_SETPOINTS> scoring;
    public final SendableChooser<Constants.Intake.SETPOINTS> intakesp;

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
        scoring.setDefaultOption("IDLE", Constants.SCORING_SETPOINTS.IDLE);
        scoring.addOption("IDLE", Constants.SCORING_SETPOINTS.IDLE);
        scoring.addOption("L1", Constants.SCORING_SETPOINTS.L1);
        scoring.addOption("L2", Constants.SCORING_SETPOINTS.L2);
        scoring.addOption("L3", Constants.SCORING_SETPOINTS.L3);
        scoring.addOption("L4", Constants.SCORING_SETPOINTS.L4);

        intakesp = new SendableChooser<>();
        intakesp.addOption("INTAKE", Constants.Intake.SETPOINTS.CORAL_PICKUP);
        intakesp.setDefaultOption("IDLE", Constants.Intake.SETPOINTS.IDLE);

        scoring.onChange((Constants.SCORING_SETPOINTS setpoint) -> {
            System.out.println("Setpoint Changing");

            if (manipulator.coralIn() && setpoint == Constants.SCORING_SETPOINTS.L4) {
                System.out.println("Setpoint Changing to reflect l4 slection");

                l4HoldManipulator().schedule();
            }
        });

        SmartDashboard.putData(scoring);

        SmartDashboard.putData(intakesp);

        // Get Named Commands inputed for Auto
        NamedCommands.registerCommand("score", elevatorScore(Constants.SCORING_SETPOINTS.L3));
        NamedCommands.registerCommand("intake", startIntaking());

        // Auto Selecter
        autoChooser = AutoBuilder.buildAutoChooser();    
        autoChooser.addOption("none", new InstantCommand(() -> {}));
        autoChooser.addOption("One Score", new PathPlannerAuto("One Piece"));
        autoChooser.addOption("One Score Far", new PathPlannerAuto("One Score Far"));
        autoChooser.addOption("line_2meters", new PathPlannerAuto("line_2meters"));
        autoChooser.addOption("square", new PathPlannerAuto("square"));
        autoChooser.addOption("test", new PathPlannerAuto("test"));

        PathPlannerLogging.setLogActivePathCallback(swerve::setAutoTrajector);

        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerve.setDefaultCommand(new TeleopSwerveDrive(swerve, oi));

        intaking = false;
        configureBindings();
    }

    public boolean getIntaking() {
        return intaking;
    }

    
    public void turnOfIntaking() {
        intaking = false;
    }

    private void configureBindings() {
        oi.getDriverButton(XboxController.Button.kY.value).onTrue(startIntaking());
        oi.getDriverButton(XboxController.Button.kStart.value).onTrue(stopIntaking());
        oi.getDriverButton(XboxController.Button.kRightBumper.value).onTrue(swerve.resetGyroCommand());
        // oi.getDriverButton(XboxController.Button.kB.value).onTrue(elevator.moveElev());
        // oi.getDriverButton(XboxController.Button.kX.value).onTrue(elevator.reZero());

        // oi.getDriverButton(XboxController.Button.kB.value).onTrue(elevator.moveUp(0.4)).onFalse(elevator.stop());
        // oi.getDriverButton(XboxController.Button.kX.value).onTrue(elevator.moveDown(0.4)).onFalse(elevator.stop());

        // oi.getDriverButton(XboxController.Button.kA.value).onTrue(manipulator.reverse());



        oi.getDriverButton(XboxController.Button.kA.value).onTrue(new InstantCommand(() -> {
            elevatorScore(scoring.getSelected()).schedule();
        }));

        oi.getDriverButton(XboxController.Button.kLeftBumper.value).onTrue(manipulator.reverse()).onFalse(manipulator.stop());

        //oi.getDriverButton(XboxController.Button.kX.value).onTrue(intake.moveWrist(intakesp.getSelected().getSetpoint(), Constants.Intake.MAX_ERROR));
    }

    public Command togleIntake() {
        // return new InstantCommand(() -> {
        //     intaking = !intaking;
        //     if(intaking){
        //         startIntaking().();;
        //     }
        //     else{
        //         stopIntaking().schedule();;
        //     }
        // });
        intaking = !intaking;
        if(intaking){
            return startIntaking();
        }
        return stopIntaking();
    }

    public Command startIntaking() {
        return new InstantCommand(() -> {this.intaking = true; SmartDashboard.putString("test", "no work");})
            .andThen(manipulator.moveWrist(Constants.SCORING_SETPOINTS.IDLE.getManipulator(), Constants.Manipulator.ERROR)).
            andThen(intake.moveWrist(Constants.Intake.SETPOINTS.CORAL_PICKUP.getSetpoint(), Constants.Intake.MAX_ERROR)).
            andThen(manipulator.run(Constants.Manipulator.SPEEDS.INTAKE)).
            andThen(indexer.run()).
            andThen(intake.run());
    }

    public Command stopIntaking() {
        intaking = false;
       return intake.moveWrist(Constants.Intake.SETPOINTS.IDLE.getSetpoint(),Constants.Intake.MAX_ERROR).
            andThen(
                new ParallelCommandGroup(
                    manipulator.stop(),
                    indexer.stop(),
                    intake.stop()
                )
        );
    }

    public Command l4HoldManipulator() {
        return manipulator.moveWrist(Constants.SCORING_SETPOINTS.L4Hold.getManipulator(), Constants.Manipulator.ERROR);
    }

    public Command elevatorScore(Constants.SCORING_SETPOINTS setpoint) {
        return new InstantCommand(() -> System.out.println("Setpoint Moving to  " + setpoint.toString())).
            andThen(elevator.moveElevator(setpoint.getElevator(), Constants.Elevator.MAX_ERROR).
            alongWith(manipulator.moveWrist(setpoint.getManipulator(), Constants.Manipulator.ERROR)).
            andThen(
                setpoint == Constants.SCORING_SETPOINTS.L4 ? 
                manipulator.reverse(Constants.Manipulator.SPEEDS.L4) : 
                manipulator.run()
            ).
            andThen(new WaitCommand(1.0)).
            andThen(manipulator.stop()).
            andThen(elevator.moveElevator(Constants.SCORING_SETPOINTS.IDLE.getElevator(), Constants.Elevator.MAX_ERROR))
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
