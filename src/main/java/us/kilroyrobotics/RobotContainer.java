// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import us.kilroyrobotics.Constants.CoralMechanismConstants;
import us.kilroyrobotics.Constants.DriveConstants;
import us.kilroyrobotics.Constants.ElevatorConstants;
import us.kilroyrobotics.generated.TunerConstants;
import us.kilroyrobotics.subsystems.CommandSwerveDrivetrain;
import us.kilroyrobotics.subsystems.CoralIntakeMotor;
import us.kilroyrobotics.subsystems.CoralIntakeMotor.CoralState;
import us.kilroyrobotics.subsystems.Elevator;
import us.kilroyrobotics.subsystems.Wrist;

public class RobotContainer {
    private double kMaxSpeed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double kMaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
    // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(kMaxSpeed * 0.1)
                    .withRotationalDeadband(kMaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(
                            DriveRequestType
                                    .OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(kMaxSpeed);

    /* Controllers */
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandJoystick leftOperatorJoystick = new CommandJoystick(1);
    private final CommandJoystick rightOperatorJoystick = new CommandJoystick(2);

    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final CoralIntakeMotor coralIntakeMotor = new CoralIntakeMotor();

    @Logged(name = "Elevator")
    public final Elevator elevator = new Elevator();

    @Logged(name = "Wrist")
    public final Wrist wrist = new Wrist(elevator::getCarriagePose, false);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    /* Coral Intake Wheel Commands */
    private Command setCoralIntaking =
            Commands.runOnce(
                    () -> coralIntakeMotor.setCoralState(CoralState.INTAKING), coralIntakeMotor);
    private Command setCoralOuttaking =
            Commands.runOnce(
                    () -> coralIntakeMotor.setCoralState(CoralState.OUTTAKING), coralIntakeMotor);
    private Command setCoralHolding =
            Commands.runOnce(
                    () -> coralIntakeMotor.setCoralState(CoralState.HOLDING), coralIntakeMotor);
    private Command setCoralOff =
            Commands.runOnce(
                    () -> coralIntakeMotor.setCoralState(CoralState.OFF), coralIntakeMotor);

    private Command elevatorSetL1 =
            Commands.runOnce(() -> elevator.setPosition(ElevatorConstants.kL1Height), elevator);
    private Command elevatorSetL2 =
            Commands.runOnce(() -> elevator.setPosition(ElevatorConstants.kL2Height), elevator);
    private Command elevatorSetL3 =
            Commands.runOnce(() -> elevator.setPosition(ElevatorConstants.kL3Height), elevator);
    private Command elevatorSetL4 =
            Commands.runOnce(() -> elevator.setPosition(ElevatorConstants.kL4Height), elevator);
    private Command elevatorSetCoralStation =
            Commands.runOnce(
                    () -> elevator.setPosition(ElevatorConstants.kCoralStationHeight), elevator);

    /* Wrist Commands */
    private Command wristSetL1 =
            Commands.runOnce(() -> wrist.setAngle(CoralMechanismConstants.kScoringL1), wrist);
    private Command wristSetL2 =
            Commands.runOnce(() -> wrist.setAngle(CoralMechanismConstants.kScoringL2), wrist);
    private Command wristSetL3 =
            Commands.runOnce(() -> wrist.setAngle(CoralMechanismConstants.kScoringL3), wrist);
    private Command wristSetL4 =
            Commands.runOnce(() -> wrist.setAngle(CoralMechanismConstants.kScoringL4), wrist);
    private Command wristSetCoralStation =
            Commands.runOnce(() -> wrist.setAngle(CoralMechanismConstants.kIntakingAngle), wrist);

    /* Preset Commands */
    private Command coralIntakeSetL1 = Commands.parallel(elevatorSetL1, wristSetL1);
    private Command coralIntakeSetL2 = Commands.parallel(elevatorSetL2, wristSetL2);
    private Command coralIntakeSetL3 = Commands.parallel(elevatorSetL3, wristSetL3);
    private Command coralIntakeSetL4 = Commands.parallel(elevatorSetL4, wristSetL4);
    private Command coralIntakeSetCoralStation =
            Commands.parallel(elevatorSetCoralStation, wristSetCoralStation);

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive.withVelocityX(
                                                -driverController.getLeftY()
                                                        * kMaxSpeed) // Drive forward with
                                        // negative Y
                                        // (forward)
                                        .withVelocityY(
                                                -driverController.getLeftX()
                                                        * kMaxSpeed) // Drive left with
                                        // negative X
                                        // (left)
                                        .withRotationalRate(
                                                -driverController.getRightX()
                                                        * kMaxAngularRate) // Drive counterclockwise
                        // with
                        // negative X (left)
                        ));

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController
                .b()
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        point.withModuleDirection(
                                                new Rotation2d(
                                                        -driverController.getLeftY(),
                                                        -driverController.getLeftX()))));

        driverController
                .pov(0)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(DriveConstants.kLowDriveSpeed)
                                                .withVelocityY(0)));
        driverController
                .pov(90)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(0)
                                                .withVelocityY(
                                                        DriveConstants.kLowDriveSpeed
                                                                .unaryMinus())));
        driverController
                .pov(180)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(
                                                        DriveConstants.kLowDriveSpeed.unaryMinus())
                                                .withVelocityY(0)));
        driverController
                .pov(270)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(0)
                                                .withVelocityY(DriveConstants.kLowDriveSpeed)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController
                .back()
                .and(driverController.y())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController
                .back()
                .and(driverController.x())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController
                .start()
                .and(driverController.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController
                .start()
                .and(driverController.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press
        driverController
                .leftBumper()
                .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Coral Intake Motor Controls
        leftOperatorJoystick.button(3).onTrue(setCoralIntaking).onFalse(setCoralHolding);
        leftOperatorJoystick.button(4).onTrue(setCoralOuttaking).onFalse(setCoralOff);

        // Wrist Control
        leftOperatorJoystick
                .button(1)
                .whileTrue(
                        Commands.run(() -> wrist.set(leftOperatorJoystick.getY() * 0.25), wrist));

        // Elevator Controls
        rightOperatorJoystick.button(10).onTrue(coralIntakeSetL1);
        rightOperatorJoystick.button(7).onTrue(coralIntakeSetL2);
        rightOperatorJoystick.button(11).onTrue(coralIntakeSetL3);
        rightOperatorJoystick.button(6).onTrue(coralIntakeSetL4);
        rightOperatorJoystick.button(8).onTrue(coralIntakeSetCoralStation);
        rightOperatorJoystick
                .button(1)
                .whileTrue(
                        Commands.run(
                                () -> elevator.set(rightOperatorJoystick.getY() * 0.25), elevator));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
