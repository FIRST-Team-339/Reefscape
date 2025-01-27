// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class Wrist extends SubsystemBase {
    private Supplier<Pose3d> getCarriagePose;

    /** Creates a new Wrist. */
    public Wrist(Supplier<Pose3d> carriagePoseGetter) {
        this.getCarriagePose = carriagePoseGetter;
    }

    @Logged(name = "WristPose")
    public Pose3d getWristPose() {
        return new Pose3d(
                0.300609,
                0.0254,
                this.getCarriagePose.get().getZ() + 0.2899918,
                new Rotation3d(
                        Degrees.of(0), Degrees.of(0) /* Wrist angle goes here */, Degrees.of(0)));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
