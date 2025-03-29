// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import us.kilroyrobotics.Constants.CoralMechanismConstants;

public class CoralIntakeMotor extends SubsystemBase {
    private SparkMax wheelMotor;
    private DigitalInput coralSensor = new DigitalInput(0);

    /** Creates a new CoralIntakeMotor. */
    public CoralIntakeMotor() {
        wheelMotor = new SparkMax(CoralMechanismConstants.kWheelMotorId, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        wheelMotor.set(speed);
    }

    public DigitalInput getCoralSensor() {
        return coralSensor;
    }

    public boolean isCoralDetected() {
        return coralSensor.get();
    }

    @Override
    public void periodic() {}
}
