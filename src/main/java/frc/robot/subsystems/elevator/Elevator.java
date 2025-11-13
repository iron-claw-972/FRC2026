// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IdConstants;
import frc.robot.util.AngledElevatorSim;
import frc.robot.util.PhoenixUtil;

/**
 * Elevator subsystem controls a linear motion mechanism (vertical lift).
 * Uses a TalonFX motor with Motion Magic for smooth motion profiling.
 * Supports both real and simulated operation.
 */
public class Elevator extends SubsystemBase {

    // Elevator motor contoller
    private TalonFX rightMotor = new TalonFX(IdConstants.ELEVATOR_RIGHT_MOTOR, Constants.CANIVORE_CAN);

    // Target elevator position
    private double setpoint = ElevatorConstants.INTAKE_SETPOINT;

    // Motion Magic control request for smooth PID + motion profile control
    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

    // Max motion constraints
    private double maxVelocity = 3.6; // m/s 3.68
    private double maxAcceleration = 14; // m/s 8

    // Sim variables
    private AngledElevatorSim sim;
    private Mechanism2d mechanism;
    private MechanismLigament2d ligament;
    private double voltage;

    // Struct used for logging sensor and state data
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    // Used to check if arm is safely stowed before lowering elevator
    private BooleanSupplier armStowed;

    /** Creates a new Elevator subsystem. */
    public Elevator() {
        // Initialize simulation tools if running in a simulated environment
        // This increases both the time and memory efficiency of the code when running
        // on a real robot; do not remove this if statement
        SmartDashboard.putData("raise 1 meter", new InstantCommand(() -> setSetpoint(1.0)));
        SmartDashboard.putData("0 meter", new InstantCommand(() -> setSetpoint(0)));
        
        if (RobotBase.isSimulation()) {
            sim = new AngledElevatorSim(ElevatorConstants.MOTOR, ElevatorConstants.GEARING,
                    ElevatorConstants.CARRIAGE_MASS,
                    ElevatorConstants.DRUM_RADIUS, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT, true,
                    ElevatorConstants.START_HEIGHT, ElevatorConstants.ANGLE, ElevatorConstants.SPRING_FORCE);
            double width = ElevatorConstants.MAX_HEIGHT * Math.sin(ElevatorConstants.ANGLE);
            double height = ElevatorConstants.MAX_HEIGHT * Math.cos(ElevatorConstants.ANGLE);
            double size = Math.max(width, height);
            mechanism = new Mechanism2d(size, size);
            ligament = mechanism.getRoot("base", size / 2 - width / 2, size / 2 - height / 2)
                    .append(new MechanismLigament2d(
                            "elevator", ElevatorConstants.START_HEIGHT,
                            90 - Units.radiansToDegrees(Math.abs(ElevatorConstants.ANGLE))));
            SmartDashboard.putData("elevator", mechanism);

            // Initialize simulation motor position to match starting height
            // Account for motor inversion: positive sim position = negative motor rotations
            double startingRotations = -ElevatorConstants.GEARING * ElevatorConstants.START_HEIGHT
                    / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS);
            rightMotor.getSimState().setRawRotorPosition(startingRotations);
        }
        Timer.delay(1.0);

        // m_lastProfiledReference = new ExponentialProfile.State(getPosition(),0);
        resetEncoder(ElevatorConstants.START_HEIGHT);

        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.15; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.75; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.GEARING * maxVelocity
                / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS); // Target cruise velocity
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.GEARING * maxAcceleration
                / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS); // Target acceleration
        rightMotor.getConfigurator().apply(talonFXConfigs);
        rightMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        updateInputs();
        PhoenixUtil.tryUntilOk(100, () -> rightMotor.setNeutralMode(NeutralModeValue.Brake));
    }

    /** Sets the arm stowed condition supplier (for safety interlock). */
    public void setArmStowed(BooleanSupplier armStowed) {
        this.armStowed = armStowed;
    }

    /** Runs every 20ms. Handles motor commands and state logging. */
    @Override
    public void periodic() {
        double setpoint2 = setpoint;
        // if (setpoint2 < ElevatorConstants.SAFE_SETPOINT && (armStowed == null || !armStowed.getAsBoolean())) {
        //     setpoint2 = ElevatorConstants.SAFE_SETPOINT;
        // }
        double setpointRotations = ElevatorConstants.GEARING * setpoint2
                / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS);
        rightMotor.setControl(voltageRequest.withPosition(setpointRotations).withFeedForward(0.4));
        updateInputs();
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator/Setpoint", getSetpoint());
        Logger.recordOutput("Elevator/AtSetpoint", atSetpoint());
    }

    /** Runs only in simulation. Updates virtual physics. */
    @Override
    public void simulationPeriodic() {
        // Get actual voltage being applied by the Motion Magic controller
        double appliedVoltage = rightMotor.getMotorVoltage().getValueAsDouble();
        sim.setInputVoltage(appliedVoltage);
        sim.update(Constants.LOOP_TIME);

        // Update visualization
        ligament.setLength(sim.getPositionMeters());

        // Update motor simulation state: convert sim position to motor rotations
        // Account for motor inversion: positive sim position = negative motor rotations
        double motorRotations = -ElevatorConstants.GEARING * sim.getPositionMeters()
                / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS);
        rightMotor.getSimState().setRawRotorPosition(motorRotations);
    }

    /** Resets the encoder position to a given height (meters). */
    public void resetEncoder(double height) {
        // Without the if statement, this causes loop overruns in simulation, and this
        // code does nothing anyway on sim (it sets the position to itself)
        if (RobotBase.isReal()) {
            rightMotor.setPosition(height / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS) * ElevatorConstants.GEARING);
        }
    }

    /** Updates telemetry input struct from hardware sensors. */
    public void updateInputs() {
        inputs.measuredPosition = rightMotor.getPosition().getValueAsDouble() / ElevatorConstants.GEARING
                * (2 * Math.PI * ElevatorConstants.DRUM_RADIUS);
        inputs.velocity = rightMotor.getVelocity().getValueAsDouble() / ElevatorConstants.GEARING
                * (2 * Math.PI * ElevatorConstants.DRUM_RADIUS);
        inputs.currentAmps = rightMotor.getStatorCurrent().getValueAsDouble();
        inputs.appliedVoltage = rightMotor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Get the position of the elevator in meters.
     */
    public double getPosition() {
        return inputs.measuredPosition;
    }

    /**
     * Get the velocity of the elevator in m/s.
     */
    public double getVelocity() {
        return inputs.velocity;
    }

    public double getAppliedVoltage() {
        return rightMotor.getMotorVoltage().getValueAsDouble();
    }

    /** @return Current applied voltage to motor in V. */
    public double getVoltage() {
        return voltage;
    }

    /**
     * Method to set the setpoint of the elevator. Clamped between min and max
     * height.
     *
     * @param setpoint The setpoint in meters.
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = MathUtil.clamp(setpoint, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT);
    }

    /**
     * Get the velocity of the elevator in meters.
     */
    public double getSetpoint() {
        return setpoint;
    }

    /** @return Mechanism2d object for simulation visualization. */
    public Mechanism2d getMechanism2d() {
        return mechanism;
    }

    /**
     * @return True if elevator is close enough to its setpoint.
     *         The tolerance is roughly ±0.0375 meters.
     */
    public boolean atSetpoint() {
        return Math.abs(getPosition() - setpoint) < (0.025 + 0.0125);
    }

    /**
     * Get the COM at the current elevater height.
     */
    public double getCenterOfMassHeight() {
        return (getPosition() - ElevatorConstants.MIN_HEIGHT)
                / (ElevatorConstants.MAX_HEIGHT - ElevatorConstants.MIN_HEIGHT)
                * (ElevatorConstants.CENTER_OF_MASS_HEIGHT_EXTENDED - ElevatorConstants.CENTER_OF_MASS_HEIGHT_STOWED)
                + ElevatorConstants.CENTER_OF_MASS_HEIGHT_STOWED;
    }
}
