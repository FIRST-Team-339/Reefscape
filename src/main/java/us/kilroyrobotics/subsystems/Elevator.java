// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import us.kilroyrobotics.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private SparkMax leadMotor;
    private SparkMax followerMotor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController pidController;

    /* Sim Specific */
    private DCMotor simElevatorGearbox;
    private SparkMaxSim simMotors;
    private ElevatorSim simElevator;

    /** Creates a new Elevator. */
    public Elevator() {
        this.leadMotor = new SparkMax(ElevatorConstants.kLeftMotorId, MotorType.kBrushless);
        this.followerMotor = new SparkMax(ElevatorConstants.kRightMotorId, MotorType.kBrushless);
        this.encoder = this.leadMotor.getEncoder();
        this.pidController = this.leadMotor.getClosedLoopController();

        // Configure
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(1.0, 0.0, 0.0, 0.0);

        this.leadMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.followerMotor.configure(
                new SparkMaxConfig().follow(this.leadMotor, true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Sim
        this.simElevatorGearbox = DCMotor.getNEO(2);
        this.simMotors = new SparkMaxSim(leadMotor, simElevatorGearbox);
        this.simElevator =
                new ElevatorSim(
                        this.simElevatorGearbox,
                        10.0,
                        Units.inchesToMeters(2.0),
                        4.0,
                        0.0,
                        0.8763,
                        true,
                        0,
                        0.1,
                        0);
    }

    public void setPosition(Distance distance) {
        this.pidController.setReference(distance.in(Inches), ControlType.kPosition);
        System.out.println(this.encoder.getPosition());
    }

    public void resetPosition() {
        encoder.setPosition(0);
    }

    public void stop() {
        this.leadMotor.setVoltage(0.0);
    }

    @Logged(name = "SecondStagePose")
    public Pose3d getSecondStagePose() {
        return new Pose3d(0, 0, Inches.of(this.encoder.getPosition()).in(Meters), new Rotation3d());
    }

    @Logged(name = "CarriagePose")
    public Pose3d getCarriagePose() {
        return this.getSecondStagePose().times(2);
    }

    @Override
    public void simulationPeriodic() {
        this.simElevator.setInput(this.simMotors.getAppliedOutput() * RoboRioSim.getVInVoltage());
        this.simElevator.update(0.02);

        this.simMotors.iterate(
                this.simElevator.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02);
    }
}
