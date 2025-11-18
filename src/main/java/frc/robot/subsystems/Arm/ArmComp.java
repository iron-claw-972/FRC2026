package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.util.PhoenixUtil;

public class ArmComp extends ArmBase {

    private static final DCMotor simMotor = DCMotor.getKrakenX60(1);
    private TalonFX motor = new TalonFX(IdConstants.ARM_MOTOR);
    //private PIDController pid = new PIDController(0.1, 0, 0);
    private ArmFeedforward ff = new ArmFeedforward(0,ArmConstants.MASS * ArmConstants.CENTER_OF_MASS_LENGTH/ArmConstants.GEAR_RATIO/ArmConstants.MOTOR.KtNMPerAmp*ArmConstants.MOTOR.rOhms, 0);
    private TalonFXSimState encoderSim = motor.getSimState(); 

    private SingleJointedArmSim armSim;

    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

    /**
     * Setpoint of the arm in degrees 
     */
    private double setpoint; 

    /**in degrees*/
    //private double setpoint = ArmConstants.START_ANGLE;

    // sim? motor idk which one to use
    // simulation Objects

    double gearRatio = ArmConstants.GEAR_RATIO;

    public ArmComp() {
        // tell the PID object the tolerance
        //pid.setTolerance(Units.degreesToRadians(ArmConstants.TOLERANCE));

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;

        slot0Configs.kS = 0;
        // torque of gravity / gear ratio
        slot0Configs.kG = ((ArmConstants.MASS * 9.81 * ArmConstants.CENTER_OF_MASS_LENGTH ) / gearRatio);
        slot0Configs.kV = 0.12;
        slot0Configs.kA = 0.0;
        slot0Configs.kP = 0.1 * 2 * Math.PI; // in rotations, not radians like the normal p
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Units.radiansToRotations(ArmConstants.MAX_VELOCITY * gearRatio);
        motionMagicConfigs.MotionMagicAcceleration = Units.radiansToRotations(ArmConstants.MAX_ACCELERATION * gearRatio);

        motor.getConfigurator().apply(talonFXConfigs);
        motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)); 
        

        // setting the motor encoder starting position to 0 degrees, but the arm is at the start angle
        motor.setPosition(0);
    

        // Tell the PID object what the setpoint is
        // Note: at power on, the encoder will be zero
        // Ideally, the PID target value should also be zero
        // because if we were to call getAngle() right now, it would be zero rather than
        // START_ANGLE.
        
        setSetpoint(ArmConstants.START_ANGLE); 
        // simulation Arm
        armSim = new SingleJointedArmSim(
            simMotor, 
            gearRatio,
            // moment of inertia
            0.0261057394, 
            0.276,
            Units.degreesToRadians(-110), 
            Units.degreesToRadians(360),
            true,
            Units.degreesToRadians(ArmConstants.START_ANGLE));
            
        PhoenixUtil.tryUntilOk(100, ()->motor.setNeutralMode(NeutralModeValue.Brake));
        // Puts the PID tuner
        //SmartDashboard.putData("PID", pid);
        
    }

    @Override
    public void periodic() {
        System.out.println("Arm at setpoint: " + atSetpoint());
        System.out.println("Arm setpoint: " + setpoint);
        System.out.println("Arm angle: " + getAngle()); 
        System.out.println("Difference: " + Math.abs(setpoint - getAngle())); 

        // Obtain the motor position
        double position = getAngle();
        // PID calculation
        //double powerPID = pid.calculate(Units.degreesToRadians(position));
        // set motor power to the result of the PID calculation
        //motor.set(powerPID + ff.calculate(Units.degreesToRadians(position),0));

        //double setpointRotations = Units.degreesToRotations(setpoint) * gearRatio;
        
        // display the current position of the arm
        displayPosition(position);
        SmartDashboard.putNumber("arm angle", getAngle());
    }

    @Override
    public void simulationPeriodic() {
        // Get the drive to the motor (motor voltage)
        double voltsMotor = motor.get() * 12; 
        // tell simulation motor what the applied motor voltage is
        armSim.setInputVoltage(voltsMotor);

        // Simulate the system for one time step 
        armSim.update(Constants.LOOP_TIME);

        // Get the arm angle after simulation is done
        double simAngle = armSim.getAngleRads(); 
        double simAngleDegrees = Units.radiansToDegrees(simAngle);

        // adjust angle offset
        double armAngleWithOffset = simAngleDegrees - ArmConstants.START_ANGLE;
        double simRotations = Units.degreesToRotations(armAngleWithOffset);

        double motorRotations = simRotations * gearRatio;
        
        // Turning the encoder by the set rotations
        encoderSim.setRawRotorPosition(motorRotations);
     }

    /**
     * Set the target position (angle) of the arm.
     * 
     * @param setpoint The target angle in degrees
     */
    @Override
    public void setSetpoint(double setpoint) {
        // Tell the PID object what thsete setpoint is
        // PID is using radians
        //pid.setSetpoint(Units.degreesToRadians(setpoint));
        this.setpoint = setpoint; 
        double setpointAdjusted = (setpoint - ArmConstants.START_ANGLE) * gearRatio;
        motor.setControl(voltageRequest.withPosition(Units.degreesToRotations(setpointAdjusted)).withFeedForward(ff.calculate(Units.degreesToRadians(getAngle()),0)));
    }

    /** Gets the arm angle in degrees */
    @Override
    public double getAngle() {
        // when the encoder reads zero, we are at START_ANGLE degrees
        return ArmConstants.START_ANGLE
                + (Units.rotationsToDegrees(motor.getPosition().getValueAsDouble())) / gearRatio;
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(setpoint - getAngle()) < ArmConstants.TOLERANCE;
        //return pid.atSetpoint();
        // return Math.abs(Units.radiansToDegrees(motor.getPosition().getValueAsDouble())) < 3.0;
    }

    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public boolean canMoveElevator() {
        return true;
    }

}