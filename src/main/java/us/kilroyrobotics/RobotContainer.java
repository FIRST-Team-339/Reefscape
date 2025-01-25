// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import us.kilroyrobotics.Constants.DriveConstants;
import us.kilroyrobotics.Constants.ElevatorConstants;
import us.kilroyrobotics.generated.TunerConstants;
import us.kilroyrobotics.subsystems.CommandSwerveDrivetrain;
import us.kilroyrobotics.subsystems.Elevator;

public class RobotContainer {
    private double kTeleopMaxSpeed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double kMaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
    // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(kTeleopMaxSpeed * 0.1)
                    .withRotationalDeadband(kMaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(
                            DriveRequestType
                                    .OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(kTeleopMaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandJoystick leftOperatorJoystick = new CommandJoystick(1);
    private final CommandJoystick rightOperatorJoystick = new CommandJoystick(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Elevator elevatorSubsystem = new Elevator();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    /** Toggles the max speed via a one-time command */
    public Command toggleMaxSpeed =
            Commands.runOnce(
                    () -> {
                        if (this.kTeleopMaxSpeed
                                == TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) {
                            this.kTeleopMaxSpeed =
                                    DriveConstants.kLowDriveSpeed.in(MetersPerSecond);
                        } else {
                            this.kTeleopMaxSpeed =
                                    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                        }
                    });

        public Command elevatorSetL1 = Commands.runOnce(
                    () ->
                        elevatorSubsystem.setPosition(ElevatorConstants.kL1Height)
                    , elevatorSubsystem);
        public Command elevatorSetL2 = Commands.runOnce(
                    () ->
                        elevatorSubsystem.setPosition(ElevatorConstants.kL2Height)
                    , elevatorSubsystem);
        public Command elevatorSetL3 = Commands.runOnce(
                    () ->
                        elevatorSubsystem.setPosition(ElevatorConstants.kL3Height)
                    , elevatorSubsystem);
        public Command elevatorSetL4 = Commands.runOnce(
                    () ->
                        elevatorSubsystem.setPosition(ElevatorConstants.kL4Height)
                    , elevatorSubsystem);
        public Command elevatorSetCoralStation = Commands.runOnce(
                    () ->
                        elevatorSubsystem.setPosition(ElevatorConstants.kCoralStationHeight)
                    , elevatorSubsystem);

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive.withVelocityX(
                                                -driverController.getLeftY()
                                                        * kTeleopMaxSpeed) // Drive forward with
                                        // negative Y
                                        // (forward)
                                        .withVelocityY(
                                                -driverController.getLeftX()
                                                        * kTeleopMaxSpeed) // Drive left with
                                        // negative X
                                        // (left)
                                        .withRotationalRate(
                                                -driverController.getRightX()
                                                        * kMaxAngularRate) // Drive counterclockwise
                        // with
                        // negative X (left)
                        ));

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b()
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        point.withModuleDirection(
                                                new Rotation2d(
                                                        -driverController.getLeftY(),
                                                        -driverController.getLeftX()))));

        driverController.pov(0)
                .whileTrue(
                        drivetrain.applyRequest(
                                () -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        driverController.pov(180)
                .whileTrue(
                        drivetrain.applyRequest(
                                () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start()
                .and(driverController.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start()
                .and(driverController.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Toggle between high and low speeds
        driverController.x().onTrue(toggleMaxSpeed);

        rightOperatorJoystick.button(1).onTrue(elevatorSetL1);
        rightOperatorJoystick.button(2).onTrue(elevatorSetL2);
        rightOperatorJoystick.button(3).onTrue(elevatorSetL3);
        rightOperatorJoystick.button(4).onTrue(elevatorSetL4);
        rightOperatorJoystick.button(5).onTrue(elevatorSetCoralStation);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
