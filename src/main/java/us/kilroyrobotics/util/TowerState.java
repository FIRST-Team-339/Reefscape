// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.util;

public enum TowerState {
    INIT,
    HOMING_ELEVATOR,
    HOMING_WRIST,
    HOME,
    RAISING_TO_INTAKE,
    TILTING_TO_INTAKE,
    INTAKING,
    GOT_CORAL,
    ALIGNING,
    ALIGNED,
    RAISING_TO_L1,
    RAISING_TO_L2,
    RAISING_TO_L3,
    RAISING_TO_L4,
    POINT_WHEELS_BACK,
    BACKUP_BEFORE_SCORE,
    WAIT_FOR_BACKUP,
    TILTING_TO_SCORE,
    TELEOP_WAIT,
    READY_TO_SCORE,
    SCORING
}
