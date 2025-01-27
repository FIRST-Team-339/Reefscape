// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import us.kilroyrobotics.Constants.CoralMechanismConstants;

public class CoralIntakeMotor extends SubsystemBase {
    private SparkMax wheelMotor;

    /** Creates a new CoralIntakeMotor. */
    public CoralIntakeMotor() {
        this.wheelMotor = new SparkMax(CoralMechanismConstants.kWheelMotorId, MotorType.kBrushless);
    }

    CoralState coralState = CoralState.OFF;

    public static enum CoralState { // 4 State Enum
        OFF,
        INTAKING,
        OUTTAKING,
        HOLDING,
    }

    public void setCoralState(CoralState newState) {
        this.coralState = newState;
    }

    @Override
    public void periodic() {
        switch (coralState) {
            case INTAKING:
                this.wheelMotor.set(CoralMechanismConstants.kWheelSpeedIntaking);
                break;
            case OUTTAKING:
                this.wheelMotor.set(CoralMechanismConstants.kWheelSpeedOuttaking);
                break;
            case HOLDING:
                this.wheelMotor.set(CoralMechanismConstants.kWheelSpeedHolding);
                break;
            default:
                this.wheelMotor.set(0);
                break;
        }
    }
}
