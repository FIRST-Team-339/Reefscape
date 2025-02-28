// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.Supplier;
import us.kilroyrobotics.Constants.CameraConstants;
import us.kilroyrobotics.Constants.CoralMechanismConstants;
import us.kilroyrobotics.Constants.DriveConstants;
import us.kilroyrobotics.Constants.ElevatorConstants;
import us.kilroyrobotics.Constants.VisionConstants;
import us.kilroyrobotics.generated.TunerConstants;
import us.kilroyrobotics.subsystems.Camera;
// import us.kilroyrobotics.subsystems.AlgaeIntake;
// import us.kilroyrobotics.subsystems.AlgaeIntake.AlgaeState;
import us.kilroyrobotics.subsystems.CommandSwerveDrivetrain;
import us.kilroyrobotics.subsystems.CoralIntakeMotor;
import us.kilroyrobotics.subsystems.CoralIntakeMotor.CoralState;
import us.kilroyrobotics.subsystems.Elevator;
import us.kilroyrobotics.subsystems.Wrist;
import us.kilroyrobotics.util.LimelightHelpers;
import us.kilroyrobotics.util.LimelightHelpers.RawFiducial;

public class RobotContainer {
    private double kMaxAngularRate =
            RotationsPerSecond.of(0.25).in(RadiansPerSecond); // 1/4 of a rotation per second
    // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(DriveConstants.kTeleopMaxSpeed.in(MetersPerSecond) * 0.1)
                    .withRotationalDeadband(kMaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(
                            DriveRequestType
                                    .OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger =
            new Telemetry(DriveConstants.kTeleopMaxSpeed.in(MetersPerSecond));

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
    public final Wrist wrist = new Wrist(elevator::getCarriagePose, Robot.isReal());

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        if (Robot.isReal() && CameraConstants.kCameraEnabled) new Camera();

        NamedCommands.registerCommand("CoralIntake", setCoralIntaking());
        NamedCommands.registerCommand("CoralOuttake", setCoralOuttaking());
        NamedCommands.registerCommand("CoralHolding", genCoralHoldingCommand());
        NamedCommands.registerCommand("CoralOff", genCoralOffCommand());

        NamedCommands.registerCommand("ElevatorL1", elevatorSetL1);
        NamedCommands.registerCommand("ElevatorL2", elevatorSetL2);
        NamedCommands.registerCommand("ElevatorL3", elevatorSetL3);
        NamedCommands.registerCommand("ElevatorL4", elevatorSetL4);
        NamedCommands.registerCommand("ElevatorCS", elevatorSetCoralStation);

        NamedCommands.registerCommand("WristL1", wristSetL1);
        NamedCommands.registerCommand("WristL2", wristSetL2);
        NamedCommands.registerCommand("WristL3", wristSetL3);
        NamedCommands.registerCommand("WristL4", wristSetL4);
        NamedCommands.registerCommand("WristCS", wristSetCoralStation);

        NamedCommands.registerCommand("Leave", autoLeave);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private Command waitToSlowDown(Supplier<Double> velocityGetter, double velocityThreshold) {
        return Commands.waitUntil(() -> Math.abs(velocityGetter.get()) < velocityThreshold);
    }

    /* Coral Intake Wheel Commands */
    private Command setCoralIntaking() {
        return new InstantCommand(
                () -> coralIntakeMotor.setCoralState(CoralState.INTAKING), coralIntakeMotor);
    }

    private Command setCoralOuttaking() {
        return new InstantCommand(
                () -> coralIntakeMotor.setCoralState(CoralState.OUTTAKING), coralIntakeMotor);
    }

    private Command genCoralHoldingCommand() {
        return new InstantCommand(
                () -> coralIntakeMotor.setCoralState(CoralState.HOLDING), coralIntakeMotor);
    }

    private Command genCoralOffCommand() {
        return new InstantCommand(
                () -> coralIntakeMotor.setCoralState(CoralState.OFF), coralIntakeMotor);
    }

    /* Elevator Commands */
    private Command elevatorSetL1 =
            Commands.runOnce(
                    () -> elevator.setPosition(ElevatorConstants.kL1Height), elevator, wrist);
    private Command elevatorSetL2 =
            Commands.runOnce(
                    () -> elevator.setPosition(ElevatorConstants.kL2Height), elevator, wrist);
    private Command elevatorSetL3 =
            Commands.runOnce(
                    () -> elevator.setPosition(ElevatorConstants.kL3Height), elevator, wrist);
    private Command elevatorSetL4 =
            Commands.runOnce(
                    () -> elevator.setPosition(ElevatorConstants.kL4Height), elevator, wrist);
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
    public Command wristSetCoralStation =
            Commands.runOnce(() -> wrist.setAngle(CoralMechanismConstants.kIntakingAngle), wrist);

    private Command wristStop() {
        return new InstantCommand(
                () -> {
                    wrist.setAngle(wrist.getAngle());
                    wrist.stop();
                },
                wrist);
    }

    /* Preset Commands */
    private Command elevatorStop =
            Commands.runOnce(
                    () -> {
                        elevator.setPosition(elevator.getPosition());
                    },
                    elevator);

    private int currentAprilTag = 0;

    private Command alignReef(boolean leftSide) {
        return drivetrain.applyRequest(
                () -> {
                    VisionConstants.rotationalPID.enableContinuousInput(-Math.PI, Math.PI);
                    VisionConstants.xPID.setIZone(0.1);
                    VisionConstants.yPID.setIZone(0.1);
                    VisionConstants.rotationalPID.setIZone(5);
                    Pose2d targetPose;

                    RawFiducial[] aprilTags = LimelightHelpers.getRawFiducials("limelight-right");
                    if (aprilTags.length < 1)
                        aprilTags = LimelightHelpers.getRawFiducials("limelight-left");

                    if (aprilTags.length < 1) {
                        if (currentAprilTag == 0) return null;

                        targetPose =
                                VisionConstants.getAlignmentPose(
                                        currentAprilTag,
                                        leftSide,
                                        DriverStation.getAlliance().orElse(Alliance.Blue));
                    } else {
                        RawFiducial aprilTag = aprilTags[0];
                        currentAprilTag = aprilTag.id;

                        targetPose =
                                VisionConstants.getAlignmentPose(
                                        aprilTag.id,
                                        leftSide,
                                        DriverStation.getAlliance().orElse(Alliance.Blue));
                    }

                    if (targetPose == null) return null;

                    if (this.drivetrain.isAtPose(targetPose)) {
                        currentAprilTag = 0;
                        System.out.println("AT POSE!");
                        return null;
                    }

                    Pose2d currentPose = drivetrain.getState().Pose;

                    double xVelocity =
                            VisionConstants.xPID.calculate(currentPose.getX(), targetPose.getX());
                    double yVelocity =
                            VisionConstants.yPID.calculate(currentPose.getY(), targetPose.getY());
                    double rotationalVelocity =
                            VisionConstants.rotationalPID.calculate(
                                    currentPose.getRotation().getRadians(),
                                    targetPose.getRotation().getRadians());

                    return drive.withVelocityX(xVelocity)
                            .withVelocityY(yVelocity)
                            .withRotationalRate(rotationalVelocity);
                });
    }

    private Timer autoLeaveTimer = new Timer();
    private Command autoLeave =
            Commands.sequence(
                            Commands.runOnce(() -> this.autoLeaveTimer.restart()),
                            drivetrain.applyRequest(
                                    () -> forwardStraight.withVelocityX(MetersPerSecond.of(-0.5))))
                    .onlyWhile(() -> !autoLeaveTimer.hasElapsed(2));

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive.withVelocityX(
                                                -driverController.getLeftY()
                                                        * DriveConstants.kTeleopMaxSpeed.in(
                                                                MetersPerSecond)) // Drive forward
                                        // with
                                        // negative Y
                                        // (forward)
                                        .withVelocityY(
                                                -driverController.getLeftX()
                                                        * DriveConstants.kTeleopMaxSpeed.in(
                                                                MetersPerSecond)) // Drive left with
                                        // negative X
                                        // (left)
                                        .withRotationalRate(
                                                -driverController.getRightX()
                                                        * kMaxAngularRate) // Drive counterclockwise
                        // with
                        // negative X (left)
                        ));

        // Brake
        driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));

        driverController
                .pov(0)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(DriveConstants.kLowDriveSpeed)
                                                .withVelocityY(0)));
        driverController
                .pov(45)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(DriveConstants.kLowDriveSpeed)
                                                .withVelocityY(
                                                        DriveConstants.kLowDriveSpeed
                                                                .unaryMinus())));
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
                .pov(135)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(
                                                        DriveConstants.kLowDriveSpeed.unaryMinus())
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
                .pov(225)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(
                                                        DriveConstants.kLowDriveSpeed.unaryMinus())
                                                .withVelocityY(DriveConstants.kLowDriveSpeed)));
        driverController
                .pov(270)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(0)
                                                .withVelocityY(DriveConstants.kLowDriveSpeed)));
        driverController
                .pov(315)
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        forwardStraight
                                                .withVelocityX(DriveConstants.kLowDriveSpeed)
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
        driverController.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Coral Intake Motor Controls
        leftOperatorJoystick.button(2).onTrue(setCoralIntaking()).onFalse(genCoralHoldingCommand());
        leftOperatorJoystick.button(3).onTrue(setCoralOuttaking()).onFalse(genCoralOffCommand());

        // Wrist Control
        leftOperatorJoystick.button(10).onTrue(wristSetL1);
        leftOperatorJoystick.button(7).onTrue(wristSetL2);
        leftOperatorJoystick.button(11).onTrue(wristSetL3);
        // leftOperatorJoystick.button(6).onTrue(wristSetL4AndStop);
        leftOperatorJoystick.button(8).onTrue(wristSetCoralStation);
        leftOperatorJoystick
                .button(1)
                .whileTrue(
                        Commands.run(
                                () ->
                                        wrist.setSpeed(
                                                -leftOperatorJoystick.getY()
                                                        * CoralMechanismConstants
                                                                .kOverrideSpeedMultiplier),
                                wrist))
                .onFalse(wristStop());

        // Elevator Controls
        rightOperatorJoystick.button(10).onTrue(elevatorSetL1);
        rightOperatorJoystick.button(7).onTrue(elevatorSetL2);
        rightOperatorJoystick.button(11).onTrue(elevatorSetL3);
        // rightOperatorJoystick.button(6).onTrue(elevatorSetL4);
        rightOperatorJoystick.button(8).onTrue(elevatorSetCoralStation);
        rightOperatorJoystick
                .button(1)
                .whileTrue(
                        Commands.run(
                                () ->
                                        elevator.setSpeed(
                                                rightOperatorJoystick.getY()
                                                        * ElevatorConstants
                                                                .kOverrideSpeedMultiplier),
                                elevator,
                                wrist))
                .onFalse(elevatorStop);

        // Reef Alignment
        driverController.leftBumper().whileTrue(alignReef(true));
        driverController.rightBumper().whileTrue(alignReef(false));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
