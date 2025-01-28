// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import us.kilroyrobotics.Constants.ElevatorConstants;
import us.kilroyrobotics.Constants.SimulationConstants;

public class Elevator extends SubsystemBase {
    private SparkMax m_leadMotor;
    private SparkMax m_followerMotor;
    private RelativeEncoder m_encoder;
    private SparkClosedLoopController m_pidController;

    /* Sim Specific */
    private DCMotor m_simElevatorGearbox;
    private SparkMaxSim m_simLeadMotor;
    private ElevatorSim m_simElevator;

    /** Creates a new Elevator. */
    public Elevator() {
        this.m_leadMotor = new SparkMax(ElevatorConstants.kLeftMotorId, MotorType.kBrushless);
        this.m_followerMotor = new SparkMax(ElevatorConstants.kRightMotorId, MotorType.kBrushless);
        this.m_pidController = this.m_leadMotor.getClosedLoopController();
        this.m_encoder = this.m_leadMotor.getEncoder();

        // Configure
        SparkMaxConfig leadMotorConfig = new SparkMaxConfig();
        leadMotorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(
                        ElevatorConstants.kP,
                        ElevatorConstants.kI,
                        ElevatorConstants.kD,
                        ElevatorConstants.kF);
        leadMotorConfig.idleMode(IdleMode.kBrake);
        leadMotorConfig.smartCurrentLimit(40);
        leadMotorConfig.encoder.positionConversionFactor(
                ElevatorConstants.kEncoderPositionConversionFactor);

        SparkMaxConfig followerMotorConfig = new SparkMaxConfig();
        followerMotorConfig.idleMode(IdleMode.kBrake);
        followerMotorConfig.smartCurrentLimit(40);
        followerMotorConfig.follow(this.m_leadMotor, true);

        this.m_leadMotor.configure(
                leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.m_followerMotor.configure(
                followerMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Sim
        this.m_simElevatorGearbox = DCMotor.getNEO(2);
        this.m_simLeadMotor = new SparkMaxSim(m_leadMotor, m_simElevatorGearbox);
        this.m_simElevator =
                new ElevatorSim(
                        this.m_simElevatorGearbox,
                        SimulationConstants.kElevatorGearing,
                        SimulationConstants.kElevatorCarriageMass.magnitude(),
                        SimulationConstants.kElevatorDrumRadius.magnitude(),
                        SimulationConstants.kElevatorMinHeight.magnitude(),
                        SimulationConstants.kElevatorMaxHeight.magnitude(),
                        true,
                        SimulationConstants.kElevatorStartingHeight.magnitude());
    }

    public void setPosition(Distance distance) {
        this.m_pidController.setReference(distance.in(Meters), ControlType.kPosition);
    }

    public void resetPosition() {
        m_encoder.setPosition(0);
    }

    public void set(double speed) {
        this.m_leadMotor.set(speed);
    }

    public void stop() {
        this.m_leadMotor.setVoltage(0.0);
    }

    @Logged(name = "SecondStagePose")
    public Pose3d getSecondStagePose() {
        return new Pose3d(
                0,
                0,
                (this.m_encoder.getPosition()
                        / ElevatorConstants.kSecondStagePositionConversionFactor),
                new Rotation3d());
    }

    @Logged(name = "CarriagePose")
    public Pose3d getCarriagePose() {
        return new Pose3d(0, 0, this.m_encoder.getPosition(), new Rotation3d());
    }

    @Override
    public void simulationPeriodic() {
        this.m_simElevator.setInput(
                this.m_simLeadMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
        this.m_simElevator.update(0.02);

        // Conver the elevator's Velocity in M/s to RPM. Divide by conversion ratio to get to
        // Rotations per Second, multiple by 60 to get Rotations per Minute
        double elevatorVelocityRPM =
                m_simElevator.getVelocityMetersPerSecond()
                        * 60.0
                        / ElevatorConstants.kEncoderPositionConversionFactor;

        this.m_simLeadMotor.iterate(elevatorVelocityRPM, RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                        this.m_simElevator.getCurrentDrawAmps()));
    }
}
