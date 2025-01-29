// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import us.kilroyrobotics.Constants.CoralMechanismConstants;
import us.kilroyrobotics.Constants.SimulationConstants;

public class Wrist extends SubsystemBase {
    private SparkMax m_wristMotor;
    private Supplier<Pose3d> getCarriagePose;
    private boolean m_useAbsoluteEncoder;
    private SparkAbsoluteEncoder m_absoluteEncoder;
    private RelativeEncoder m_relativeEncoder;
    private SparkClosedLoopController m_pidController;

    /* Sim Specific */
    private DCMotor m_simWristGearbox;
    private SparkMaxSim m_simWristMotor;
    private SingleJointedArmSim m_simWrist;

    /** Creates a new Wrist. */
    public Wrist(Supplier<Pose3d> carriagePoseGetter, boolean useAbsoluteEncoder) {
        this.m_wristMotor =
                new SparkMax(CoralMechanismConstants.kWristMotorId, MotorType.kBrushless);
        this.m_pidController = this.m_wristMotor.getClosedLoopController();
        this.m_useAbsoluteEncoder = useAbsoluteEncoder;
        if (useAbsoluteEncoder) {
            this.m_absoluteEncoder = this.m_wristMotor.getAbsoluteEncoder();
        } else {
            this.m_relativeEncoder = this.m_wristMotor.getEncoder();
        }

        // Configure
        SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
        wristMotorConfig
                .closedLoop
                .feedbackSensor(
                        useAbsoluteEncoder
                                ? FeedbackSensor.kAbsoluteEncoder
                                : FeedbackSensor.kPrimaryEncoder)
                .pidf(
                        CoralMechanismConstants.kP,
                        CoralMechanismConstants.kI,
                        CoralMechanismConstants.kD,
                        CoralMechanismConstants.kF);
        wristMotorConfig.idleMode(IdleMode.kBrake);
        wristMotorConfig.smartCurrentLimit(40);

        this.m_wristMotor.configure(
                wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.getCarriagePose = carriagePoseGetter;

        // Sim
        this.m_simWristGearbox = DCMotor.getNEO(1);
        this.m_simWristMotor = new SparkMaxSim(this.m_wristMotor, m_simWristGearbox);
        this.m_simWrist =
                new SingleJointedArmSim(
                        m_simWristGearbox,
                        SimulationConstants.kWristGearing,
                        SingleJointedArmSim.estimateMOI(
                                SimulationConstants.kArmLength.magnitude(),
                                SimulationConstants.kWristMass.magnitude()),
                        SimulationConstants.kArmLength.magnitude(),
                        SimulationConstants.kMinAngle.in(Radians),
                        SimulationConstants.kMaxAngle.in(Rotations),
                        true,
                        CoralMechanismConstants.kStartingAngle.in(Radians));
    }

    public double getVelocity() {
        return this.m_wristMotor.getAppliedOutput();
    }

    public void setAngle(Angle angle) {
        this.m_pidController.setReference(angle.times(64).in(Rotations), ControlType.kPosition);
    }

    public void setSpeed(double speed) {
        this.m_wristMotor.set(speed);
    }

    public void stop() {
        this.m_wristMotor.setVoltage(0.0);
    }

    @Logged(name = "WristPose")
    public Pose3d getWristPose() {
        return new Pose3d(
                0.300609,
                0.0254,
                this.getCarriagePose.get().getZ() + 0.2899918,
                new Rotation3d(
                        Degrees.of(0),
                        Radians.of(
                                (this.m_useAbsoluteEncoder
                                                ? this.m_absoluteEncoder.getPosition()
                                                : this.m_relativeEncoder.getPosition())
                                        / 64),
                        Degrees.of(0)));
    }

    @Override
    public void simulationPeriodic() {
        this.m_simWrist.setInput(m_simWristMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
        this.m_simWrist.update(0.02);

        this.m_simWristMotor.iterate(
                Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
                        m_simWrist.getVelocityRadPerSec()),
                RoboRioSim.getVInVoltage(),
                0.02);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                        this.m_simWrist.getCurrentDrawAmps()));
    }
}
