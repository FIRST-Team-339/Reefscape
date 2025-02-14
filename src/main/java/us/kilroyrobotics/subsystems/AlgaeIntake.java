// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import us.kilroyrobotics.Constants.AlgaeConstants;

public class AlgaeIntake extends SubsystemBase {
	private SparkMax algaeMotorLeader;
	private SparkMax algaeMotorFollower;

	/** Creates a new AlgaeIntake. */
	public AlgaeIntake() {
		this.algaeMotorLeader = new SparkMax(AlgaeConstants.kAlgaeMotorLeaderId, MotorType.kBrushed);
		this.algaeMotorFollower = new SparkMax(AlgaeConstants.kAlgaeMotorFollowerId, MotorType.kBrushed);

		SparkMaxConfig leaderConfig = new SparkMaxConfig();
		leaderConfig.smartCurrentLimit(40);
		this.algaeMotorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		SparkMaxConfig followerConfig = new SparkMaxConfig();
		followerConfig.smartCurrentLimit(40);
		followerConfig.follow(this.algaeMotorLeader, true);

		this.algaeMotorFollower.configure(
				followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	AlgaeState algaeState = AlgaeState.OFF;

	public static enum AlgaeState {
		OFF,
		INTAKING,
		OUTTAKING
	}

	public void setAlgaeState(AlgaeState newState) {
		this.algaeState = newState;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		switch (algaeState) {
			case INTAKING:
				this.algaeMotorLeader.set(AlgaeConstants.kAlgaeSpeedIntaking);
				break;
			case OUTTAKING:
				this.algaeMotorLeader.set(AlgaeConstants.kAlgaeSpeedOuttaking);
				break;
			default:
				this.algaeMotorLeader.set(0);
				break;
		}
	}
}
