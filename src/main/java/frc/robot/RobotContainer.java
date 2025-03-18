// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.SCORING_SETPOINTS;
import frc.robot.Constants.AutoAlign.POSITIONS;
import frc.robot.commands.AlignToReef;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.util.RobotZoneDetector;
import poplib.controllers.oi.OI;
import poplib.controllers.oi.XboxOI;
import poplib.smart_dashboard.TunableNumber;
import poplib.swerve.commands.SysIdSwerve;
import poplib.swerve.commands.TeleopSwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Translation2d;
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
    //UsbCamera camera = CameraServer.startAutomaticCapture(0);



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

        //camera.setResolution(640, 480);


        // Scoring Selecter
        scoring = new SendableChooser<>();
        scoring.setDefaultOption("IDLE", Constants.SCORING_SETPOINTS.IDLE);
        scoring.addOption("IDLE", Constants.SCORING_SETPOINTS.IDLE);
        scoring.addOption("L1", Constants.SCORING_SETPOINTS.L1);
        scoring.addOption("L2", Constants.SCORING_SETPOINTS.L2);
        scoring.addOption("L3", Constants.SCORING_SETPOINTS.L3);
        scoring.addOption("L4", Constants.SCORING_SETPOINTS.L4);
        scoring.addOption("AlageL1", Constants.SCORING_SETPOINTS.ALGAEL1);
        scoring.addOption("AlgaeL2", Constants.SCORING_SETPOINTS.ALGAEL2);

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


        // Auto Selecter
        autoChooser = AutoBuilder.buildAutoChooser();    
        autoChooser.addOption("none", new InstantCommand(() -> {}));
        autoChooser.addOption("One Score", new PathPlannerAuto("One Piece"));
        autoChooser.addOption("One Score Far", new PathPlannerAuto("One Score Far"));
        autoChooser.addOption("line_2meters", new PathPlannerAuto("line_2meters"));
        autoChooser.addOption("square", new PathPlannerAuto("square"));
        autoChooser.addOption("test", new PathPlannerAuto("test"));
        autoChooser.addOption("center_left_score", new PathPlannerAuto("center_left_score"));
        autoChooser.addOption("systems_test", new PathPlannerAuto("systems_test"));

        // Get Named Commands inputed for Auto
        NamedCommands.registerCommand("score", elevatorScore(Constants.SCORING_SETPOINTS.L3));
        NamedCommands.registerCommand("intake", startIntaking());
        NamedCommands.registerCommand("L4_hold", l4HoldManipulator());
        NamedCommands.registerCommand("goToL4", goToScoringPosition(Constants.SCORING_SETPOINTS.L4));
        NamedCommands.registerCommand("scoreL3", goToScoringPosition(Constants.SCORING_SETPOINTS.L3));
        NamedCommands.registerCommand("launch_ coral", manipulator.run(Constants.Manipulator.SPEEDS.L4));
        NamedCommands.registerCommand("resetEle", goToScoringPosition(Constants.SCORING_SETPOINTS.IDLE));

        PathPlannerLogging.setLogActivePathCallback(swerve::setAutoTrajector);

        SmartDashboard.putData("Scoring level", scoring);
        SmartDashboard.putData("Intake position", intakesp);
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
        // Driver Controls
        oi.getDriverButton(XboxController.Button.kY.value).onTrue(startIntaking());
        oi.getDriverButton(XboxController.Button.kStart.value).onTrue(stopIntaking());
        oi.getDriverButton(XboxController.Button.kX.value).onTrue(swerve.resetGyroCommand());
        // oi.getDriverButton(XboxController.Button.kB.value).whileTrue(manipulator.reverse(Constants.Manipulator.SPEEDS.ALGAE)).whileFalse(manipulator.stop());
        oi.getDriverButton(XboxController.Button.kB.value).onTrue(manipulator.reverse()).onFalse(manipulator.stop());
        oi.getDriverButton(XboxController.Button.kA.value).onTrue(new InstantCommand(() -> {
            // elevatorScore(scoring.getSelected()).schedule();
            goToScoringPosition(scoring.getSelected()).schedule();
        }));
        oi.getDriverButton(XboxController.Button.kLeftBumper.value).onTrue(goToIdleFromL4());;
        oi.getDriverButton(XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> {
            goToScoringPosition(Constants.SCORING_SETPOINTS.L4).schedule();
        }));
        oi.getDriverTrigger(XboxController.Axis.kLeftTrigger.value).onTrue(swerve.moveToPoseVision(POSITIONS.LEFT));
        // oi.getDriverButton(XboxController.Button.kRightBumper.value).onTrue(swerve.moveToPoseVision(new Translation2d(0.0, 1.0)));

        // Operator Controls
        // oi.getOperatorButton(XboxController.Button.kA.value).onTrue(elevator.moveDown(0.1)).onFalse(elevator.stop());
        // oi.getOperatorController().

        oi.getOperatorButton(XboxController.Button.kA.value).onTrue(manipulator.autoScore(scoring.getSelected() == SCORING_SETPOINTS.L4));
        oi.getOperatorButton(XboxController.Button.kB.value).whileTrue(manipulator.reverse()).onFalse((manipulator.stop()));
        oi.getOperatorButton(XboxController.Button.kX.value).whileTrue(manipulator.run(Constants.Manipulator.SPEEDS.REVERSE)).onFalse((manipulator.stop()));
        oi.getOperatorButton(XboxController.Button.kY.value)
        .onTrue(goToScoringPosition(Constants.SCORING_SETPOINTS.IDLE).andThen(manipulator.stop()));
        oi.getOperatorButton(XboxController.Button.kRightBumper.value).onTrue(goToScoringPosition(scoring.getSelected()));
        oi.getOperatorButton(XboxController.Button.kLeftBumper.value).onTrue(goToIdleFromL4());
        oi.getOperatorrigger(XboxController.Axis.kRightTrigger.value).onTrue(manipulator.run(Constants.Manipulator.SPEEDS.L4)).onFalse(manipulator.stop());
        oi.getOperatorrigger(XboxController.Axis.kLeftTrigger.value).onTrue(manipulator.reverse(Constants.Manipulator.SPEEDS.ALGAE)).onFalse(manipulator.stop());
        // oi.getOperatorButton(XboxController.Button.kStart.value).onTrue(elevator.reZero());
        //XboxController.Axis.

        oi.getOperatorButton(XboxController.Button.kStart.value).onTrue(funnyElevator()).onFalse(goToIdleFromL4());
        
    }

    public Command funnyElevator() {
        return (goToScoringPosition(SCORING_SETPOINTS.L4).andThen(goToIdleFromL4())).repeatedly();
    }

    public Command toggleIntake() {
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
            andThen(manipulator.moveWrist(setpoint.getManipulator(), Constants.Manipulator.ERROR)).
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

    public Command goToScoringPosition(Constants.SCORING_SETPOINTS setpoint){
        return new InstantCommand(() -> System.out.println("going to setpoint " + setpoint.toString())).
            andThen(elevator.moveElevator(setpoint.getElevator(), Constants.Elevator.MAX_ERROR).
            andThen(manipulator.moveWrist(setpoint.getManipulator(), Constants.Manipulator.ERROR))
        );
    }

    public Command goToIdleFromL4(){
        return new InstantCommand(() -> System.out.println("going idle from l4")).
        // andThen(manipulator.moveWrist(SCORING_SETPOINTS.L4Hold.getManipulator(), Constants.Manipulator.ERROR)).
        // andThen(elevator.moveElevator(SCORING_SETPOINTS.L1.getElevator(), Constants.Elevator.MAX_ERROR)).
        andThen(manipulator.moveWrist(SCORING_SETPOINTS.IDLE.getManipulator(), Constants.Manipulator.ERROR)).
        andThen(elevator.moveElevator(SCORING_SETPOINTS.IDLE.getElevator(), Constants.Elevator.MAX_ERROR));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
