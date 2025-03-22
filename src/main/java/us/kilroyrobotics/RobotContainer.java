// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
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
import us.kilroyrobotics.subsystems.LEDs;
import us.kilroyrobotics.subsystems.LEDs.LEDMode;
import us.kilroyrobotics.subsystems.Wrist;
import us.kilroyrobotics.util.LimelightHelpers;
import us.kilroyrobotics.util.LimelightHelpers.RawFiducial;

public class RobotContainer {
    private LinearVelocity currentDriveSpeed = DriveConstants.kMediumDriveSpeed;
    private double kMaxAngularRate =
            RotationsPerSecond.of(0.4).in(RadiansPerSecond); // 2/5 of a rotation per second
    // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(this.currentDriveSpeed.in(MetersPerSecond) * 0.1)
                    .withRotationalDeadband(kMaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(
                            DriveRequestType
                                    .OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(this.currentDriveSpeed.in(MetersPerSecond));

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

    public final LEDs leds = new LEDs();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    @SuppressWarnings("unused")
    public RobotContainer() {
        if (Robot.isReal() && CameraConstants.kCameraEnabled) new Camera();

        NamedCommands.registerCommand("CoralIntake", setCoralIntaking);
        NamedCommands.registerCommand("CoralOuttake", setCoralOuttaking);
        NamedCommands.registerCommand("CoralHolding", genCoralHoldingCommand);
        NamedCommands.registerCommand("CoralOff", genCoralOffCommand);
        NamedCommands.registerCommand("WaitForCoral", Commands.waitTime(Seconds.of(3.5)));

        NamedCommands.registerCommand("ElevatorBottom", elevatorSetBottom);
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

        autoChooser.onChange(
                (Command command) -> {
                    if (command != null)
                        System.out.println("[AUTO] Selected Route " + command.getName());
                });

        configureBindings();
    }

    /* Coral Intake Wheel Commands */
    private Command setCoralIntaking =
            Commands.runOnce(
                    () -> {
                        coralIntakeMotor.setCoralState(CoralState.INTAKING);
                        leds.setMode(LEDMode.WaitingForCoral);
                    },
                    coralIntakeMotor,
                    leds);

    private Command setCoralOuttaking =
            Commands.runOnce(
                            () -> coralIntakeMotor.setCoralState(CoralState.OUTTAKING),
                            coralIntakeMotor)
                    .andThen(Commands.runOnce(() -> leds.setMode(LEDMode.Off), leds));

    private Command genCoralHoldingCommand =
            Commands.runOnce(
                    () -> coralIntakeMotor.setCoralState(CoralState.HOLDING), coralIntakeMotor);

    private Command genCoralOffCommand =
            Commands.runOnce(
                    () -> {
                        coralIntakeMotor.setCoralState(CoralState.OFF);
                        leds.setMode(LEDMode.Off);
                    },
                    coralIntakeMotor,
                    leds);

    private Command waitForCoral =
            Commands.waitUntil(() -> coralIntakeMotor.getCoralSensor().get())
                    .withTimeout(Seconds.of(3.5));

    /* Elevator Commands */
    private Command elevatorSetBottom =
            Commands.runOnce(() -> elevator.setPosition(ElevatorConstants.kZeroed), elevator);
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
            Commands.runOnce(() -> wrist.setAngle(CoralMechanismConstants.kScoringL2), wrist)
                    .andThen(Commands.runOnce(() -> leds.setMode(LEDMode.Rainbow), leds));
    private Command wristSetL3 =
            Commands.runOnce(() -> wrist.setAngle(CoralMechanismConstants.kScoringL3), wrist);
    private Command wristSetL4 =
            Commands.runOnce(() -> wrist.setAngle(CoralMechanismConstants.kScoringL4), wrist);
    public Command wristSetCoralStation =
            Commands.runOnce(() -> wrist.setAngle(CoralMechanismConstants.kIntakingAngle), wrist)
                    .andThen(Commands.runOnce(() -> leds.setMode(LEDMode.Rainbow), leds));

    private Command wristStop =
            Commands.runOnce(
                    () -> {
                        wrist.setAngle(wrist.getAngle());
                        wrist.stop();
                    },
                    wrist);

    /* Preset Commands */
    private Command elevatorStop =
            Commands.runOnce(
                    () -> {
                        elevator.setPosition(elevator.getPosition());
                    },
                    elevator);

    private int currentAprilTag = 0;

    private Command alignReef(boolean leftSide) {
        return Commands.runOnce(
                () -> {
                    Pose2d targetPose;

                    RawFiducial[] aprilTags = LimelightHelpers.getRawFiducials("limelight-right");
                    if (aprilTags.length < 1)
                        aprilTags = LimelightHelpers.getRawFiducials("limelight-left");

                    if (aprilTags.length < 1) {
                        if (currentAprilTag == 0) return;

                        targetPose =
                                VisionConstants.getAlignmentPose(
                                        currentAprilTag,
                                        leftSide,
                                        DriverStation.getAlliance().orElse(Alliance.Blue));
                    } else {
                        RawFiducial aprilTag = aprilTags[0];
                        currentAprilTag = aprilTag.id;
                        System.out.println(
                                "[TELEOP-ASSIST] "
                                        + " Selected tag "
                                        + this.currentAprilTag
                                        + " for alignment");

                        targetPose =
                                VisionConstants.getAlignmentPose(
                                        aprilTag.id,
                                        leftSide,
                                        DriverStation.getAlliance().orElse(Alliance.Blue));
                    }

                    if (targetPose == null) return;

                    System.out.println(
                            "[TELEOP-ASSIST] "
                                    + (leftSide ? "[LEFT]" : "[RIGHT]")
                                    + " Going to pose "
                                    + targetPose);

                    List<Waypoint> waypoints =
                            PathPlannerPath.waypointsFromPoses(
                                    this.drivetrain.getState().Pose, targetPose);

                    PathConstraints constraints = new PathConstraints(1.5, 1.0, 0.75, 0.5);

                    PathPlannerPath path =
                            new PathPlannerPath(
                                    waypoints,
                                    constraints,
                                    null,
                                    new GoalEndState(0.0, targetPose.getRotation()));
                    path.preventFlipping = true;

                    CommandScheduler.getInstance()
                            .schedule(
                                    Commands.sequence(
                                            Commands.runOnce(
                                                    () -> {
                                                        this.leds.setMode(LEDMode.Off);
                                                        SmartDashboard.putBoolean(
                                                                "TeleopAlignIndicator", false);
                                                    }),
                                            AutoBuilder.followPath(path),
                                            Commands.runOnce(
                                                    () -> {
                                                        System.out.println(
                                                                "[TELEOP-ASSIST] "
                                                                        + (leftSide
                                                                                ? "[LEFT]"
                                                                                : "[RIGHT]")
                                                                        + " Arrived at Pose for tag "
                                                                        + this.currentAprilTag);
                                                        this.currentAprilTag = 0;

                                                        SmartDashboard.putBoolean(
                                                                "TeleopAlignIndicator", true);
                                                        this.leds.setMode(LEDMode.TeleopAligned);

                                                        CommandScheduler.getInstance()
                                                                .schedule(
                                                                        Commands.sequence(
                                                                                new WaitCommand(
                                                                                        2.5),
                                                                                Commands.runOnce(
                                                                                        () -> {
                                                                                            SmartDashboard
                                                                                                    .putBoolean(
                                                                                                            "TeleopAlignIndicator",
                                                                                                            false);
                                                                                            this
                                                                                                    .leds
                                                                                                    .setMode(
                                                                                                            LEDMode
                                                                                                                    .Off);
                                                                                        })));
                                                    })));
                });
    }

    private Timer autoLeaveTimer = new Timer();
    private Command autoLeave =
            Commands.sequence(
                            Commands.runOnce(() -> this.autoLeaveTimer.restart()),
                            drivetrain.applyRequest(
                                    () -> forwardStraight.withVelocityX(MetersPerSecond.of(0.5))))
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
                                                        * this.currentDriveSpeed.in(
                                                                MetersPerSecond)) // Drive forward
                                        // with
                                        // negative Y
                                        // (forward)
                                        .withVelocityY(
                                                -driverController.getLeftX()
                                                        * this.currentDriveSpeed.in(
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
        // driverController
        //         .back()
        //         .and(driverController.y())
        //         .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController
        //         .back()
        //         .and(driverController.x())
        //         .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController
        //         .start()
        //         .and(driverController.y())
        //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController
        //         .start()
        //         .and(driverController.x())
        //         .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press
        driverController.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Switch to higher speeds (defense mode)
        driverController
                .start()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    boolean defenseModeOn =
                                            SmartDashboard.getBoolean("DefenseModeOn", true);
                                    this.currentDriveSpeed =
                                            defenseModeOn
                                                    ? DriveConstants.kMediumDriveSpeed
                                                    : DriveConstants.kHighDriveSpeed;
                                    SmartDashboard.putBoolean("DefenseModeOn", !defenseModeOn);
                                    if (defenseModeOn) {
                                        leds.setMode(LEDMode.Defense);
                                    } else leds.setMode(LEDMode.Off);
                                }));

        // Coral Intake Motor Controls
        leftOperatorJoystick.button(2).onTrue(setCoralIntaking).onFalse(genCoralHoldingCommand);
        leftOperatorJoystick.button(3).onTrue(setCoralOuttaking).onFalse(genCoralOffCommand);

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
                .onFalse(wristStop);

        // Elevator Controls
        rightOperatorJoystick.button(9).onTrue(elevatorSetBottom);
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
        driverController.leftBumper().onTrue(alignReef(true));
        driverController.rightBumper().onTrue(alignReef(false));

        drivetrain.registerTelemetry(logger::telemeterize);

        new Trigger(this.coralIntakeMotor::isCoralDetected)
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    this.leds.setMode(LEDMode.CoralDetected);
                                    SmartDashboard.putBoolean("CoralDetected", true);
                                },
                                leds))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    this.leds.setMode(LEDMode.Off);
                                    SmartDashboard.putBoolean("CoralDetected", false);
                                },
                                leds));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
