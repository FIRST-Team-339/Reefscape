// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import us.kilroyrobotics.Constants.CoralMechanismConstants;
import us.kilroyrobotics.Constants.ElevatorConstants;
import us.kilroyrobotics.Constants.VisionConstants;
import us.kilroyrobotics.subsystems.LEDs.LEDMode;
import us.kilroyrobotics.util.LimelightHelpers;
import us.kilroyrobotics.util.LimelightHelpers.RawFiducial;
import us.kilroyrobotics.util.TowerEvent;
import us.kilroyrobotics.util.TowerState;

public class Tower extends SubsystemBase {

    private TowerState currentState = TowerState.INIT;
    private TowerEvent pendingEvent = TowerEvent.NONE;

    private final Timer stateTimer = new Timer();

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.PointWheelsAt point;
    private final Elevator elevator;
    private final Wrist wrist;
    private final CoralIntakeMotor coralIntakeMotor;
    private final LEDs leds;

    private final Command backupCommand;

    private int currentLevel = 0;

    /** Creates a new Tower. */
    public Tower(
            CommandSwerveDrivetrain drivetrain,
            SwerveRequest.RobotCentric forwardStraight,
            SwerveRequest.PointWheelsAt point,
            Elevator elevator,
            Wrist wrist,
            CoralIntakeMotor coralIntakeMotor,
            LEDs leds) {
        this.drivetrain = drivetrain;
        this.point = point;
        this.elevator = elevator;
        this.wrist = wrist;
        this.coralIntakeMotor = coralIntakeMotor;
        this.leds = leds;

        backupCommand =
                Commands.runEnd(
                                () ->
                                        drivetrain.setControl(
                                                forwardStraight
                                                        .withVelocityX(FeetPerSecond.of(-1.5))
                                                        .withVelocityY(0)),
                                () -> drivetrain.setControl(forwardStraight.withVelocityX(0)))
                        .withTimeout(0.3);

        stateTimer.start();
    }

    public void initialize() {
        if (DriverStation.isAutonomousEnabled())
            coralIntakeMotor.setSpeed(CoralMechanismConstants.kWheelSpeedHolding);
        else coralIntakeMotor.setSpeed(0);

        elevator.stop();
        elevator.resetClosedLoopControl();

        wrist.stop();
        wrist.resetClosedLoopControl();

        pendingEvent = TowerEvent.NONE;
        setState(TowerState.INIT);
    }

    /* Reef Alignment */
    private int currentAprilTag = 0;
    private Command alignmentCommand;

    private Command alignReef(boolean leftSide) {
        Pose2d targetPose;

        ArrayList<RawFiducial> aprilTags =
                new ArrayList<>(
                        Arrays.asList(
                                        LimelightHelpers.getRawFiducials("limelight-right"),
                                        LimelightHelpers.getRawFiducials("limelight-left"))
                                .stream()
                                .flatMap(Arrays::stream)
                                .toList());

        if (aprilTags.size() < 1) {
            if (currentAprilTag == 0) return null;

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

        if (targetPose == null) return null;

        System.out.println(
                "[TELEOP-ASSIST] "
                        + (leftSide ? "[LEFT]" : "[RIGHT]")
                        + " Going to pose "
                        + targetPose);

        List<Waypoint> waypoints =
                PathPlannerPath.waypointsFromPoses(this.drivetrain.getState().Pose, targetPose);

        PathConstraints constraints = new PathConstraints(1.5, 1.0, 0.75, 0.5);

        PathPlannerPath path =
                new PathPlannerPath(
                        waypoints,
                        constraints,
                        null,
                        new GoalEndState(0.0, targetPose.getRotation()));
        path.preventFlipping = true;

        return Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    this.leds.setMode(LEDMode.Off);
                                    SmartDashboard.putBoolean("TeleopAlignIndicator", false);
                                }),
                        AutoBuilder.followPath(path),
                        Commands.runOnce(
                                () -> {
                                    System.out.println(
                                            "[TELEOP-ASSIST] "
                                                    + (leftSide ? "[LEFT]" : "[RIGHT]")
                                                    + " Arrived at Pose for tag "
                                                    + this.currentAprilTag);
                                    this.currentAprilTag = 0;
                                    setState(TowerState.ALIGNED);

                                    SmartDashboard.putBoolean("TeleopAlignIndicator", true);
                                    this.leds.setMode(LEDMode.TeleopAligned);
                                }))
                .withTimeout(Seconds.of(10));
    }

    public void runStateMachine() {
        if (isTriggered(TowerEvent.HOME_TOWER)) setState(TowerState.INIT);
        if (isTriggered(TowerEvent.SCORE_BYPASS)) {
            coralIntakeMotor.setSpeed(CoralMechanismConstants.kWheelSpeedOuttaking);

            setState(TowerState.SCORING);
        }
        switch (currentState) {
            case INIT:
                wrist.setAngle(CoralMechanismConstants.kIntakingAngle);
                coralIntakeMotor.setSpeed(CoralMechanismConstants.kWheelSpeedHolding);

                setState(TowerState.HOMING_WRIST);
                break;
            case HOMING_WRIST:
                if (wrist.inPosition()) setState(TowerState.HOMING_ELEVATOR);
                break;
            case HOMING_ELEVATOR:
                if (elevator.inPosition()) {
                    elevator.setPosition(ElevatorConstants.kZeroed);

                    setState(TowerState.HOME);
                }
                break;
            case HOME:
                if (isTriggered(TowerEvent.INTAKE_CORAL)) {
                    wrist.setAngle(CoralMechanismConstants.kIntakingAngle);

                    setState(TowerState.TILTING_TO_INTAKE);
                } else if (coralIntakeMotor.isCoralDetected()) setState(TowerState.GOT_CORAL);
                else currentLevel = 0;
                break;
            case TILTING_TO_INTAKE:
                if (wrist.inPosition()) {
                    elevator.setPosition(ElevatorConstants.kCoralStationHeight);

                    setState(TowerState.RAISING_TO_INTAKE);
                }
                break;
            case RAISING_TO_INTAKE:
                if (elevator.inPosition()) {
                    coralIntakeMotor.setSpeed(CoralMechanismConstants.kWheelSpeedIntaking);
                    leds.setMode(LEDMode.WaitingForCoral);
                    SmartDashboard.putBoolean("CoralDetected", false);

                    setState(TowerState.INTAKING);
                }
                break;
            case INTAKING:
                if (coralIntakeMotor.isCoralDetected() || isTriggered(TowerEvent.INTAKE_BYPASS)) {
                    // elevator.setPosition(ElevatorConstants.kZeroed);
                    coralIntakeMotor.setSpeed(CoralMechanismConstants.kWheelSpeedHolding);
                    leds.setMode(LEDMode.CoralDetected);
                    SmartDashboard.putBoolean("CoralDetected", true);

                    setState(TowerState.GOT_CORAL);
                }
                break;
            case GOT_CORAL:
                if (DriverStation.isAutonomousEnabled()) {
                    setState(TowerState.ALIGNED);
                    break;
                }
                if (isTriggered(TowerEvent.ALIGN_BYPASS)) {
                    if (alignmentCommand != null) {
                        alignmentCommand.end(true);

                        System.out.println("[TELEOP-ASSIST] Alignment BYPASSED");
                        this.currentAprilTag = 0;
                        setState(TowerState.ALIGNED);

                        SmartDashboard.putBoolean("TeleopAlignIndicator", true);
                        this.leds.setMode(LEDMode.TeleopAligned);
                    }
                    setState(TowerState.ALIGNED);
                } else if (isTriggered(TowerEvent.ALIGN_LEFT)) {
                    alignmentCommand = alignReef(true);

                    if (alignmentCommand != null) {
                        CommandScheduler.getInstance().schedule(alignmentCommand);

                        setState(TowerState.ALIGNING);
                    }
                } else if (isTriggered(TowerEvent.ALIGN_RIGHT)) {
                    alignmentCommand = alignReef(false);

                    if (alignmentCommand != null) {
                        CommandScheduler.getInstance().schedule(alignmentCommand);

                        setState(TowerState.ALIGNING);
                    }
                }
                break;
            case ALIGNING:
                if (isTriggered(TowerEvent.CANCEL_ALIGNMENT)) {
                    alignmentCommand.cancel();
                    setState(TowerState.GOT_CORAL);
                } else if (isTriggered(TowerEvent.ALIGN_BYPASS)) {
                    alignmentCommand.end(true);
                    setState(TowerState.ALIGNED);
                }
                break;
            case ALIGNED:
                TowerEvent pendingAlignmentEvent = pendingEvent;
                if (isTriggered(TowerEvent.ALIGN_LEFT) || isTriggered(TowerEvent.ALIGN_RIGHT)) {
                    pendingEvent = pendingAlignmentEvent;
                    setState(TowerState.GOT_CORAL);
                    break;
                }
                if (isTriggered(TowerEvent.GOTO_L1)) {
                    elevator.setPosition(ElevatorConstants.kL1Height);
                    coralIntakeMotor.setSpeed(CoralMechanismConstants.kWheelSpeedHolding);

                    currentLevel = 1;
                    setState(TowerState.RAISING_TO_L1);
                } else if (isTriggered(TowerEvent.GOTO_L2)) {
                    elevator.setPosition(ElevatorConstants.kL2Height);
                    wrist.setAngle(CoralMechanismConstants.kScoringL2);
                    coralIntakeMotor.setSpeed(CoralMechanismConstants.kWheelSpeedHolding);

                    currentLevel = 2;
                    setState(TowerState.RAISING_TO_L2);
                } else if (isTriggered(TowerEvent.GOTO_L3)) {
                    elevator.setPosition(ElevatorConstants.kL3Height);
                    coralIntakeMotor.setSpeed(CoralMechanismConstants.kWheelSpeedHolding);

                    currentLevel = 3;
                    setState(TowerState.RAISING_TO_L3);
                } else if (isTriggered(TowerEvent.GOTO_L4)) {
                    elevator.setPosition(ElevatorConstants.kL4Height);
                    coralIntakeMotor.setSpeed(CoralMechanismConstants.kWheelSpeedHolding);

                    currentLevel = 4;
                    setState(TowerState.POINT_WHEELS_BACK);
                }
                break;
            case RAISING_TO_L1:
                if (elevator.inPosition()) {
                    wrist.setAngle(CoralMechanismConstants.kScoringL1);

                    setState(TowerState.TILTING_TO_SCORE);
                }
                break;
            case RAISING_TO_L2:
                if (elevator.inPosition()) {
                    wrist.setAngle(CoralMechanismConstants.kScoringL2);

                    setState(TowerState.TILTING_TO_SCORE);
                }
                break;
            case RAISING_TO_L3:
                if (elevator.inPosition()) {
                    wrist.setAngle(CoralMechanismConstants.kScoringL3);

                    setState(TowerState.TILTING_TO_SCORE);
                }
                break;
            case RAISING_TO_L4:
                if (elevator.inPosition()) {
                    wrist.setAngle(CoralMechanismConstants.kScoringL4);

                    setState(TowerState.TILTING_TO_SCORE);
                }
                break;
            case TILTING_TO_SCORE:
                if (wrist.inPosition()) {
                    if (DriverStation.isTeleopEnabled()) setState(TowerState.TELEOP_WAIT);
                    else setState(TowerState.READY_TO_SCORE);
                }
                break;
            case TELEOP_WAIT:
                if (stateTimer.hasElapsed(30)) setState(TowerState.READY_TO_SCORE);
                break;
            case READY_TO_SCORE:
                if (isTriggered(TowerEvent.SCORE_BYPASS) || stateTimer.hasElapsed(0.85)) {
                    coralIntakeMotor.setSpeed(CoralMechanismConstants.kWheelSpeedOuttaking);

                    setState(TowerState.SCORING);
                } else if ((isTriggered(TowerEvent.GOTO_L4)) && (currentLevel != 4)) {
                    elevator.setPosition(ElevatorConstants.kL4Height);
                    wrist.setAngle(CoralMechanismConstants.kScoringL4);

                    currentLevel = 4;
                    setState(TowerState.POINT_WHEELS_BACK);
                } else if ((isTriggered(TowerEvent.GOTO_L3)) && (currentLevel != 3)) {
                    elevator.setPosition(ElevatorConstants.kL3Height);
                    wrist.setAngle(CoralMechanismConstants.kScoringL3);

                    currentLevel = 3;
                    setState(TowerState.RAISING_TO_L3);
                } else if ((isTriggered(TowerEvent.GOTO_L2)) && (currentLevel != 2)) {
                    elevator.setPosition(ElevatorConstants.kL2Height);
                    wrist.setAngle(CoralMechanismConstants.kScoringL2);

                    currentLevel = 2;
                    setState(TowerState.RAISING_TO_L2);
                } else if ((isTriggered(TowerEvent.GOTO_L1)) && (currentLevel != 1)) {
                    elevator.setPosition(ElevatorConstants.kL1Height);
                    wrist.setAngle(CoralMechanismConstants.kScoringL1);

                    currentLevel = 1;
                    setState(TowerState.RAISING_TO_L1);
                }
                break;
            case POINT_WHEELS_BACK:
                CommandScheduler.getInstance()
                        .schedule(
                                Commands.runEnd(
                                                () ->
                                                        drivetrain.setControl(
                                                                point.withModuleDirection(
                                                                        Rotation2d.kZero)),
                                                () -> setState(TowerState.BACKUP_BEFORE_SCORE))
                                        .until(
                                                () ->
                                                        drivetrain.getState()
                                                                        .ModulePositions[0]
                                                                        .angle
                                                                == Rotation2d.kZero)
                                        .withTimeout(Seconds.of(0.25)));
                break;
            case BACKUP_BEFORE_SCORE:
                CommandScheduler.getInstance().schedule(backupCommand);
                setState(TowerState.WAIT_FOR_BACKUP);
                break;
            case WAIT_FOR_BACKUP:
                if (backupCommand.isFinished()) setState(TowerState.RAISING_TO_L4);
                break;
            case SCORING:
                if (!coralIntakeMotor.isCoralDetected() || stateTimer.hasElapsed(0.3)) {
                    leds.setMode(LEDMode.Off);
                    if (DriverStation.isAutonomousEnabled()) pendingEvent = TowerEvent.INTAKE_CORAL;

                    setState(TowerState.INIT);
                    SmartDashboard.putBoolean("CoralDetected", false);
                    leds.setMode(LEDMode.Off);
                    SmartDashboard.putBoolean("TeleopAlignIndicator", false);
                }
                break;
        }
    }

    public Command triggerEvent(TowerEvent event) {
        return runOnce(
                () -> {
                    System.out.println("P: " + pendingEvent + " S: " + currentState);
                    pendingEvent = event;
                });
    }

    public TowerEvent getPendingEvent() {
        return pendingEvent;
    }

    public TowerState getState() {
        return currentState;
    }

    private Boolean isTriggered(TowerEvent event) {
        if (pendingEvent == event) {
            pendingEvent = TowerEvent.NONE;
            return true;
        } else {
            return false;
        }
    }

    public void setState(TowerState newState) {
        currentState = newState;
        stateTimer.reset();
        System.out.println("P: " + pendingEvent + " S: " + currentState);
        SmartDashboard.putString("TowerState", currentState.toString());
        SmartDashboard.putString("TowerPendingEvent", pendingEvent.toString());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        runStateMachine();
    }
}
