// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {
    Orchestra orchestra = new Orchestra();

    /** Creates a new Orchestra. */
    public Music(CommandSwerveDrivetrain drivetrain) {
        orchestra.addInstrument(drivetrain.getModule(0).getDriveMotor());
        orchestra.addInstrument(drivetrain.getModule(0).getSteerMotor());
        orchestra.addInstrument(drivetrain.getModule(1).getDriveMotor());
        orchestra.addInstrument(drivetrain.getModule(1).getSteerMotor());
        orchestra.addInstrument(drivetrain.getModule(2).getDriveMotor());
        orchestra.addInstrument(drivetrain.getModule(2).getSteerMotor());
        orchestra.addInstrument(drivetrain.getModule(3).getDriveMotor());
        orchestra.addInstrument(drivetrain.getModule(3).getSteerMotor());

        orchestra.loadMusic("MrRoboto.chrp");
    }

    public void play() {
        orchestra.play();
    }

    public void stop() {
        orchestra.stop();
    }
}
