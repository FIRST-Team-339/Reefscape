// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.json.simple.parser.ParseException;
import us.kilroyrobotics.Constants.CameraConstants;
import us.kilroyrobotics.Constants.CoralMechanismConstants;
import us.kilroyrobotics.Constants.DriveConstants;
import us.kilroyrobotics.Constants.ElevatorConstants;
import us.kilroyrobotics.Constants.VisionConstants;
import us.kilroyrobotics.generated.TunerConstants;
import us.kilroyrobotics.subsystems.Camera;
import us.kilroyrobotics.subsystems.CommandSwerveDrivetrain;
import us.kilroyrobotics.subsystems.CoralIntakeMotor;
import us.kilroyrobotics.subsystems.Elevator;
import us.kilroyrobotics.subsystems.LEDs;
import us.kilroyrobotics.subsystems.LEDs.LEDMode;
import us.kilroyrobotics.subsystems.Tower;
import us.kilroyrobotics.subsystems.Wrist;
import us.kilroyrobotics.util.LimelightHelpers;
import us.kilroyrobotics.util.LimelightHelpers.RawFiducial;
import us.kilroyrobotics.util.TowerEvent;

public class RobotContainer {
    private LinearVelocity currentDriveSpeed = DriveConstants.kMediumDriveSpeed;
    private double kMaxAngularRate =
            RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 1/2 of a rotation per second
    // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(currentDriveSpeed.in(MetersPerSecond) * 0.1)
                    .withRotationalDeadband(kMaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(
                            DriveRequestType
                                    .OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry telemetry = new Telemetry(this.currentDriveSpeed.in(MetersPerSecond));

    /* Controllers */
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandJoystick operatorJoystick = new CommandJoystick(1);

    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final CoralIntakeMotor coralIntakeMotor = new CoralIntakeMotor();

    @Logged(name = "Elevator")
    public final Elevator elevator = new Elevator();

    @Logged(name = "Wrist")
    public final Wrist wrist = new Wrist(elevator::getCarriagePose, Robot.isReal());

    public final LEDs leds = new LEDs();

    public final Tower tower =
            new Tower(drivetrain, forwardStraight, point, elevator, wrist, coralIntakeMotor, leds);

    /* Drivetrain Control Commands */
    private final Command defenseMode =
            Commands.runOnce(
                    () -> {
                        boolean defenseModeOn = SmartDashboard.getBoolean("DefenseModeOn", true);
                        currentDriveSpeed =
                                defenseModeOn
                                        ? DriveConstants.kMediumDriveSpeed
                                        : DriveConstants.kHighDriveSpeed;
                        SmartDashboard.putBoolean("DefenseModeOn", !defenseModeOn);
                    });

    private int currentAprilTag = 0;

    /* Tower Commands */
    private final Command towerIntakeCoral = tower.triggerEvent(TowerEvent.INTAKE_CORAL);
    private final Command towerIntakeBypass = tower.triggerEvent(TowerEvent.INTAKE_BYPASS);
    private final Command towerScoreCoral = tower.triggerEvent(TowerEvent.SCORE_BYPASS);
    private final Command towerToHome = tower.triggerEvent(TowerEvent.HOME_TOWER);
    private final Command towerToL1 = tower.triggerEvent(TowerEvent.GOTO_L1);
    private final Command towerToL2 = tower.triggerEvent(TowerEvent.GOTO_L2);
    private final Command towerToL3 = tower.triggerEvent(TowerEvent.GOTO_L3);
    private final Command towerToL4 = tower.triggerEvent(TowerEvent.GOTO_L4);

    private Command elevatorStop =
            elevator.runOnce(() -> elevator.setPosition(elevator.getPosition()));

    private Command wristStop = wrist.runOnce(() -> wrist.setAngle(wrist.getAngle()));

    public final Command controlElevatorManual =
            elevator.run(
                    () ->
                            elevator.setSpeed(
                                    operatorJoystick.getY()
                                            * ElevatorConstants.kOverrideSpeedMultiplier));

    public final Command controlWristManual =
            wrist.run(
                    () ->
                            wrist.setSpeed(
                                    -operatorJoystick.getY()
                                            * CoralMechanismConstants.kOverrideSpeedMultiplier));

    /* Reef Alignment */
    private Command alignReef(boolean leftSide) {
        return Commands.runOnce(
                () -> {
                    Pose2d targetPose;

                    ArrayList<RawFiducial> aprilTags =
                            new ArrayList<>(
                                    Arrays.asList(
                                                    LimelightHelpers.getRawFiducials(
                                                            "limelight-right"),
                                                    LimelightHelpers.getRawFiducials(
                                                            "limelight-left"))
                                            .stream()
                                            .flatMap(Arrays::stream)
                                            .toList());

                    if (aprilTags.size() < 1) {
                        if (currentAprilTag == 0) return;

                        targetPose =
                                VisionConstants.getAlignmentPose(
                                        currentAprilTag,
                                        leftSide,
                                        DriverStation.getAlliance().orElse(Alliance.Blue));
                    } else {
                        aprilTags.sort((a, b) -> Double.compare(b.distToRobot, a.distToRobot));

                        RawFiducial aprilTag = aprilTags.get(0);
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

    /* AutoLeave Command */
    private final Timer autoLeaveTimer = new Timer();
    private final Command autoLeave =
            Commands.sequence(
                            Commands.runOnce(() -> autoLeaveTimer.restart()),
                            drivetrain.applyRequest(
                                    () -> forwardStraight.withVelocityX(MetersPerSecond.of(0.5))))
                    .onlyWhile(() -> !autoLeaveTimer.hasElapsed(2));

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    @SuppressWarnings("unused")
    public RobotContainer() {
        if (Robot.isReal() && CameraConstants.kCameraEnabled) new Camera();

        NamedCommands.registerCommand(
                "Intake Coral",
                tower.triggerEvent(TowerEvent.INTAKE_CORAL)
                        .andThen(Commands.waitTime(Seconds.of(3)))
                        .until(() -> coralIntakeMotor.isCoralDetected()));
        NamedCommands.registerCommand("Score Coral", towerScoreCoral);
        NamedCommands.registerCommand(
                "GoTo L1",
                tower.triggerEvent(TowerEvent.GOTO_L1).andThen(Commands.waitTime(Seconds.of(1.5))));
        NamedCommands.registerCommand(
                "GoTo L2",
                tower.triggerEvent(TowerEvent.GOTO_L2).andThen(Commands.waitTime(Seconds.of(1.5))));
        NamedCommands.registerCommand(
                "GoTo L3",
                tower.triggerEvent(TowerEvent.GOTO_L3).andThen(Commands.waitTime(Seconds.of(1.5))));
        NamedCommands.registerCommand(
                "GoTo L4",
                tower.triggerEvent(TowerEvent.GOTO_L4).andThen(Commands.waitTime(Seconds.of(1.5))));

        NamedCommands.registerCommand("Leave", autoLeave);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        autoChooser.onChange(
                (Command command) -> {
                    if (command != null) {
                        System.out.println("[AUTO] Selected Route " + command.getName());
                        if (command.getName().equals("InstantCommand"))
                            telemetry.displayFullAutoPath(null);
                        try {
                            telemetry.displayFullAutoPath(
                                    PathPlannerAuto.getPathGroupFromAutoFile(command.getName()));
                        } catch (IOException | ParseException e) {
                        }
                    }
                });

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive.withVelocityX(
                                                -driverController.getLeftY()
                                                        * currentDriveSpeed.in(
                                                                MetersPerSecond)) // Drive forward
                                        // with
                                        // negative Y
                                        // (forward)
                                        .withVelocityY(
                                                -driverController.getLeftX()
                                                        * currentDriveSpeed.in(
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
        driverController.start().onTrue(defenseMode);

        /* Automatic Control */
        operatorJoystick.button(9).onTrue(towerToHome);
        operatorJoystick.button(8).onTrue(towerIntakeCoral);
        operatorJoystick.button(10).onTrue(towerToL1);
        operatorJoystick.button(7).onTrue(towerToL2);
        operatorJoystick.button(11).onTrue(towerToL3);
        operatorJoystick.button(6).onTrue(towerToL4);
        operatorJoystick.button(2).onTrue(towerIntakeBypass);
        operatorJoystick.button(3).onTrue(towerScoreCoral);

        /* Manual Control */
        operatorJoystick.trigger().whileTrue(controlElevatorManual).onFalse(elevatorStop);
        operatorJoystick.button(4).whileTrue(controlWristManual).onFalse(wristStop);

        // Reef Alignment
        driverController.leftBumper().onTrue(alignReef(true));
        driverController.rightBumper().onTrue(alignReef(false));

        drivetrain.registerTelemetry(telemetry::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
