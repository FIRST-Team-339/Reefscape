// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import us.kilroyrobotics.Constants.AlgaeConstants;
import us.kilroyrobotics.Constants.CoralMechanismConstants;
import us.kilroyrobotics.subsystems.CoralIntakeMotor.CoralState;

public class AlgaeIntake extends SubsystemBase {
  private SparkMax algaeMotorLeader;
  private SparkMax algaeMotorFollower;

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    this.algaeMotorLeader = new SparkMax(AlgaeConstants.kAlgaeMotorLeaderId, MotorType.kBrushless);
    this.algaeMotorFollower = new SparkMax(AlgaeConstants.kAlgaeMotorFollowerId, MotorType.kBrushless);
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
          this.algaeMotorFollower.set(AlgaeConstants.kAlgaeSpeedOuttaking);
          break;
      case OUTTAKING:
          this.algaeMotorLeader.set(AlgaeConstants.kAlgaeSpeedOuttaking);
          this.algaeMotorFollower.set(AlgaeConstants.kAlgaeSpeedIntaking);
          break;
      default:
          this.algaeMotorLeader.set(0);
          this.algaeMotorFollower.set(0);
          break;
  }
  }
}
