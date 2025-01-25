// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import us.kilroyrobotics.Constants.CoralMechanismConstants;
import us.kilroyrobotics.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    /** Creates a new Elevator. */

    private SparkMax leadMotor;
    private SparkMax followerMotor;
    private RelativeEncoder encoder; 
    private SparkClosedLoopController pidController;

    public Elevator() {
        this.leadMotor = new SparkMax(ElevatorConstants.kLeftMotorId, MotorType.kBrushless);
        this.followerMotor = new SparkMax(ElevatorConstants.kRightMotorId, MotorType.kBrushless);
        this.encoder = this.leadMotor.getEncoder();
        this.pidController = this.leadMotor.getClosedLoopController();

        // Configure
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(1.0, 0.0, 0.0, 0.0);

        this.leadMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.followerMotor.configure(new SparkMaxConfig().follow(this.leadMotor, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPosition(Distance distance) {
        this.pidController.setReference(distance.in(Inches), ControlType.kPosition);
    }

    public void resetPosition() {
        encoder.setPosition(0);
    }

    public void stop() {
        this.leadMotor.setVoltage(0.0);
    }
}
