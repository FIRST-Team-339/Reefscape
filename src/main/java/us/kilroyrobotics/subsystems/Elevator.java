// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    /** Creates a new Elevator. */
    public Elevator() {}

    @Logged(name = "SecondStagePose")
    public Pose3d getSecondStagePose() {
        return new Pose3d(0, 0, Inches.of(0).in(Meters), new Rotation3d());
    }

    @Logged(name = "CarriagePose")
    public Pose3d getCarriagePose() {
        return this.getSecondStagePose().times(2);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
