// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import us.kilroyrobotics.Constants.CoralMechanismConstants;
import us.kilroyrobotics.Constants.ElevatorConstants;
import us.kilroyrobotics.Constants.VisionConstants;
import us.kilroyrobotics.subsystems.LEDs.LEDMode;
import us.kilroyrobotics.util.LimelightHelpers;

@Logged
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    @Logged(name = "RobotContainer")
    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();

        // Reset Elevator Encoder to Starting Position
        m_robotContainer.elevator.resetEncoder();

        Epilogue.bind(this);

        SmartDashboard.putNumber("AutoDelay", 0.0);
        SmartDashboard.putBoolean("DefenseModeOn", false);
        SmartDashboard.putBoolean("TeleopAlignIndicator", false);
        SmartDashboard.putBoolean("CoralDetected", false);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        /*
         * This example of adding Limelight is very simple and may not be sufficient for
         * on-field use.
         * Users typically need to provide a standard deviation that scales with the
         * distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible,
         * though exact implementation
         * of how to use vision should be tuned per-robot and to the team's
         * specification.
         */
        if (VisionConstants.kUseLimelight) {
            var driveState = m_robotContainer.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            LimelightHelpers.SetRobotOrientation("limelight-right", headingDeg, 0, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation("limelight-left", headingDeg, 0, 0, 0, 0, 0);
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
            if (llMeasurement == null)
                llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
            if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
                m_robotContainer.drivetrain.addVisionMeasurement(
                        llMeasurement.pose,
                        Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
            }
        }
    }

    @Override
    public void disabledInit() {
        this.m_robotContainer.leds.setMode(LEDMode.Rainbow);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        this.m_robotContainer.leds.setMode(LEDMode.Off);
    }

    @Override
    public void autonomousInit() {
        Shuffleboard.startRecording();
        Shuffleboard.selectTab("Autonomous");

        Command autoDelayCommand = new WaitCommand(SmartDashboard.getNumber("AutoDelay", 0.0));

        m_autonomousCommand =
                Commands.sequence(autoDelayCommand, m_robotContainer.getAutonomousCommand());

        if (m_autonomousCommand != null) {
            m_robotContainer.wristSetCoralStation.schedule();
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        Shuffleboard.selectTab("Teleop");
        m_robotContainer.wristSetCoralStation.schedule();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        Shuffleboard.stopRecording();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
        m_robotContainer.elevator.setPosition(ElevatorConstants.kCoralStationHeight);
        m_robotContainer.wrist.setAngle(CoralMechanismConstants.kIntakingAngle);
    }

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        m_robotContainer.elevator.simulationPeriodic();
        m_robotContainer.wrist.simulationPeriodic();
    }
}
