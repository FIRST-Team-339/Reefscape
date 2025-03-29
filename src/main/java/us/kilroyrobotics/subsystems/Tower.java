// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import static edu.wpi.first.units.Units.FeetPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import us.kilroyrobotics.Constants.CoralMechanismConstants;
import us.kilroyrobotics.Constants.ElevatorConstants;
import us.kilroyrobotics.subsystems.LEDs.LEDMode;
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

    private final Timer movementTimer = new Timer();
    private final Command backupCommand;

    private double safetyFactor = 1;
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
                drivetrain
                        .applyRequest(() -> forwardStraight.withVelocityX(FeetPerSecond.of(-1)))
                        .until(() -> this.movementTimer.hasElapsed((0.25)));

        stateTimer.start();
    }

    public void initialize() {
        coralIntakeMotor.setSpeed(0);

        elevator.stop();
        elevator.resetClosedLoopControl();

        wrist.stop();
        wrist.resetClosedLoopControl();

        pendingEvent = TowerEvent.NONE;
        setState(TowerState.INIT);
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
                coralIntakeMotor.setSpeed(0.0);

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

                    setState(TowerState.INTAKING);
                }
                break;
            case INTAKING:
                if (coralIntakeMotor.isCoralDetected() || isTriggered(TowerEvent.INTAKE_BYPASS)) {
                    // elevator.setPosition(ElevatorConstants.kZeroed);
                    coralIntakeMotor.setSpeed(CoralMechanismConstants.kWheelSpeedHolding);
                    leds.setMode(LEDMode.CoralDetected);

                    setState(TowerState.GOT_CORAL);
                }
                break;
            case GOT_CORAL:
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
                    setState(TowerState.RESTART_BACKUP_TIMER);
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
                if (wrist.inPosition()) setState(TowerState.READY_TO_SCORE);
                break;
            case READY_TO_SCORE:
                if (isTriggered(TowerEvent.SCORE_BYPASS) || stateTimer.hasElapsed(1.0)) {
                    coralIntakeMotor.setSpeed(CoralMechanismConstants.kWheelSpeedOuttaking);

                    setState(TowerState.SCORING);
                } else if ((isTriggered(TowerEvent.GOTO_L4)) && (currentLevel != 4)) {
                    elevator.setPosition(ElevatorConstants.kL4Height);
                    wrist.setAngle(CoralMechanismConstants.kScoringL4);

                    currentLevel = 4;
                    setState(TowerState.RESTART_BACKUP_TIMER);
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
            case RESTART_BACKUP_TIMER:
                movementTimer.restart();
                setState(TowerState.POINT_WHEELS_BACK);
                break;
            case POINT_WHEELS_BACK:
                CommandScheduler.getInstance()
                        .schedule(
                                drivetrain.applyRequest(
                                        () -> point.withModuleDirection(Rotation2d.kZero)));
                setState(TowerState.BACKUP_BEFORE_SCORE);
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
                    setState(TowerState.INIT);
                }
                break;
        }
    }

    public void triggerEvent(TowerEvent event) {
        pendingEvent = event;
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
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        runStateMachine();
    }
}
