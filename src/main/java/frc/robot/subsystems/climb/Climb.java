package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Climb extends SubsystemBase{
    private TalonFX ClimbMotor;
    private DutyCycleEncoder ClimbEncoder;

    private final MotionMagicVoltage request = new MotionMagicVoltage(0);

    private SingleJointedArmSim intakeSim;
    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("baseMotor", 25, 0));

    private double setpoint;

    private TalonFXSimState simEncoder;
    private DCMotor simMotor = DCMotor.getKrakenX60(1);

    public Climb(){
        ClimbMotor = new TalonFX(IdConstants.CLIMB_ID, Constants.RIO_CAN);
        ClimbEncoder = new DutyCycleEncoder(IdConstants.CLIMB_ENCODER_ID);

        simEncoder = ClimbMotor.getSimState();

        intakeSim = new SingleJointedArmSim(simMotor, 
            ClimbConstants.gear_ratio, 
            1, 
            0.5,
            0, 
            Units.degreesToRadians(360), 
            true, 
            0
            );

        SmartDashboard.putData("Climb" ,mechanism2d);                            

        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.0; 
        slot0Configs.kV = 0.0;
        slot0Configs.kA = 0.0;
        slot0Configs.kP = 20.0; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0.0; // no output for integrated error
        slot0Configs.kD = 0.05; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ClimbConstants.MAX_VELOCITY * ClimbConstants.gear_ratio; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = ClimbConstants.MAX_ACCELERATION * ClimbConstants.gear_ratio; // Target acceleration of 160 rps/s (0.5 seconds)

        ClimbMotor.getConfigurator().apply(talonFXConfigs);

        ClimbMotor.setPosition((Units.rotationsToDegrees(ClimbEncoder.get()) - ClimbConstants.absoluteOffsetAngle) * ClimbConstants.gear_ratio);

        SmartDashboard.putData("Climb to First Position" , new InstantCommand(() -> ClimbToFirstPosition()));
        SmartDashboard.putData("Set Climb 90 degrees", new InstantCommand(() -> setSetpoint(90))); 
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
        ClimbMotor.setControl(request.withPosition(Units.degreesToRotations(setpoint) * ClimbConstants.gear_ratio));
    }

    public boolean atSetpoint(){
        return Math.abs(setpoint - getPosition()) < ClimbConstants.tolerance;
    }

    public double getPosition(){
        return Units.rotationsToDegrees(ClimbMotor.getPosition().getValueAsDouble());
    }

    public double getEncoderValue(){
        return Units.rotationsToDegrees(ClimbEncoder.get());
    }

    public void periodic(){
        SmartDashboard.putNumber("Climb Position", getPosition());
        SmartDashboard.putNumber("Encoder Value", getEncoderValue());
    }

    public void simulationPeriodic(){
        intakeSim.setInputVoltage(ClimbMotor.getMotorVoltage().getValueAsDouble());
        ligament2d.setAngle(Units.radiansToDegrees(intakeSim.getAngleRads()));

        simEncoder.setRawRotorPosition(Units.radiansToRotations(intakeSim.getAngleRads()) * ClimbConstants.gear_ratio);

        intakeSim.update(0.02);
    }

    public void ClimbToFirstPosition(){
        setSetpoint(ClimbConstants.climbFirstStage + Units.rotationsToDegrees(ClimbMotor.getPosition().getValueAsDouble()/ClimbConstants.gear_ratio));
    }

    public void ClimbToSecondPosition(){
        setSetpoint(ClimbConstants.climbSecondStage + Units.rotationsToDegrees(ClimbMotor.getPosition().getValueAsDouble()/ClimbConstants.gear_ratio));
    }
}
