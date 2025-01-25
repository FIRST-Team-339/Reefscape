// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import us.kilroyrobotics.Constants.CoralMechanismConstants;
import us.kilroyrobotics.Constants.DriveConstants;
import us.kilroyrobotics.generated.TunerConstants;
import us.kilroyrobotics.subsystems.CoralIntake.CoralState;

public class CoralIntake extends SubsystemBase {
/** Creates a new Intake. */
    private SparkMax wheelMotor;

    public CoralIntake() {
        this.wheelMotor = new SparkMax(CoralMechanismConstants.kWheelMotorId, MotorType.kBrushless);
    }

    //stuff I (Colin) worked on Jan 23 2025. Do what you will with the following:
    CoralState coralState = CoralState.OFF;

    enum CoralState { // "4 State Enum"
        OFF,
        INTAKING,
        OUTTAKING,
        HOLDING,
    }

    public Command setIntaking =  //"Coral Intake Commands"
            Commands.runOnce(
                    () ->  {
                        coralState = CoralState.INTAKING;
                    });

    public Command setOuttaking = 
            Commands.runOnce(
                    () -> {
                        coralState = CoralState.OUTTAKING;
                    });

    public Command setHolding = 
            Commands.runOnce(
                    () -> {
                        coralState = CoralState.HOLDING;
                    });

    public Command setOff = 
            Commands.runOnce(
                    () -> {
                        coralState = CoralState.OFF;
                    });

    //and so ends my (Colin) work on Jan 23 2025. I apologize in advance.


    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
