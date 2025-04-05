// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {
    private Orchestra orchestra = new Orchestra();
    private StatusCode status;

    /** Creates a new Orchestra. */
    public Music(CommandSwerveDrivetrain drivetrain) {
        orchestra.addInstrument(drivetrain.getModule(0).getDriveMotor(), 0);
        orchestra.addInstrument(drivetrain.getModule(0).getSteerMotor(), 0);
        orchestra.addInstrument(drivetrain.getModule(1).getDriveMotor(), 0);
        orchestra.addInstrument(drivetrain.getModule(1).getSteerMotor(), 0);
        orchestra.addInstrument(drivetrain.getModule(2).getDriveMotor(), 0);
        orchestra.addInstrument(drivetrain.getModule(2).getSteerMotor(), 0);
        orchestra.addInstrument(drivetrain.getModule(3).getDriveMotor(), 0);
        orchestra.addInstrument(drivetrain.getModule(3).getSteerMotor(), 0);

        status = orchestra.loadMusic("music/mii_channel.chrp");
        System.out.println("Loading Music - " + status);

        SmartDashboard.putBoolean("PlayMusic", false);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            stop();
            return;
        }

        if (SmartDashboard.getBoolean("PlayMusic", false)) {
            play();
        } else {
            stop();
        }
    }

    public void play() {
        if (!orchestra.isPlaying()) {
            System.out.println("Playing Music - " + status);
            status = orchestra.play();
        }
    }

    public void stop() {
        if (orchestra.isPlaying()) status = orchestra.stop();
    }
}
