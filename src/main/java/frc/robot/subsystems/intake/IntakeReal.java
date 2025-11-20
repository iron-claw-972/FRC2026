package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.constants.IntakeConstants;

public class IntakeReal extends IntakeBase{
    TalonFX IntakeMotor;
    TalonFX FlywheelMotor;

    private TalonFXSimState IntakeSimEncoder;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    ArmFeedforward IntakeFeedforward;
    PIDController IntakePID;

    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("IntakeMotor", 25, 0));

    DCMotor sim_motor = DCMotor.getKrakenX60(1);

    private SingleJointedArmSim IntakeSim = 
    new SingleJointedArmSim(
        sim_motor, 
        IntakeConstants.gear_ratio, 
        IntakeConstants.momment_of_intertia, 
        IntakeConstants.arm_length, 
        0, 
        Units.degreesToRadians(360), 
        true, 
        0);    


    public IntakeReal(){
        IntakeMotor = new TalonFX(IntakeConstants.intake_motor_id);

        FlywheelMotor = new TalonFX(IntakeConstants.flywheel_motor_id); 

        IntakeSimEncoder = IntakeMotor.getSimState();

        IntakePID = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

        IntakeFeedforward = new ArmFeedforward
            (0.0,
            IntakeConstants.gravity * IntakeConstants.center_of_mass * (IntakeConstants.arm_mass)
                    / IntakeConstants.gear_ratio * sim_motor.rOhms / sim_motor.KtNMPerAmp / IntakeConstants.robot_voltage,
            0);
        
        IntakeMotor.getConfigurator().apply(getMotionMagicConfig());

        SmartDashboard.putData("IntakePID", IntakePID);
        SmartDashboard.putData("Intake", mechanism2d);

        SmartDashboard.putData("Set 0 degrees", new InstantCommand(() -> setIntakeSetpoint(0)));
        SmartDashboard.putData("Set 90 degrees", new InstantCommand(() -> setIntakeSetpoint(90)));
        SmartDashboard.putData("Set 180 degrees", new InstantCommand(() -> setIntakeSetpoint(180)));
        SmartDashboard.putData("Set 270 degrees", new InstantCommand(() -> setIntakeSetpoint(270)));
    }

    public TalonFXConfiguration getMotionMagicConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kS = 0.1; // Static friction compensation (should be >0 if friction exists)
        config.Slot0.kG = IntakeConstants.arm_mass * IntakeConstants.center_of_mass * 9.8 / IntakeConstants.gear_ratio; // Gravity compensation
        config.Slot0.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        config.Slot0.kA = 0; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        config.Slot0.kP = Units.radiansToRotations(0.5 * 12); // If position error is 2.5 rotations, apply 12V (0.5 * 2.5 * 12V)
        config.Slot0.kI = Units.radiansToRotations(0.00); // Integral term (usually left at 0 for MotionMagic)
        config.Slot0.kD = Units.radiansToRotations(0.1 * 12); // Derivative term (used to dampen oscillations)

        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Units.radiansToRotations(IntakeConstants.max_velocity) * IntakeConstants.gear_ratio;
        motionMagicConfigs.MotionMagicAcceleration = Units.radiansToRotations(IntakeConstants.max_acceleration) * IntakeConstants.gear_ratio;

        return config;
    }

    public void setIntakeSetpoint(double position){
        IntakeMotor.setControl(m_request.withPosition(Units.degreesToRotations(position*IntakeConstants.gear_ratio)));
    }
    
    public double getIntakePosition(){
        return Units.rotationsToRadians(IntakeMotor.getPosition().getValueAsDouble()/IntakeConstants.gear_ratio);
    }

    public double getIntakeVelocity(){
        return Units.rotationsPerMinuteToRadiansPerSecond(IntakeMotor.getVelocity().getValueAsDouble() * 60);
    }

    public boolean IntakeAtSetpoint(){
        return Math.abs( Units.rotationsToRadians(m_request.Position) - getIntakePosition() ) < IntakeConstants.position_tolerance;
    }

    public void setFlywheel(double power){
        FlywheelMotor.set(power);
    }

    public double getFlywheelVelocity(){
        return Units.rotationsPerMinuteToRadiansPerSecond(FlywheelMotor.getVelocity().getValueAsDouble() * 60);
    }

    public void stopFlywheel(){
        FlywheelMotor.stopMotor();
    }

    public void setMomentOfInertia(double momentOfIntertia){
        var system = LinearSystemId.createSingleJointedArmSystem(sim_motor, momentOfIntertia, IntakeConstants.gear_ratio);
        // IntakeSim.setState(new Matrix<sim_motor, N1>(new SimpleMatrix(system.getA().getNumRows(), 1)));
    }

    @Override
    public void periodic() {
        ligament2d.setAngle(Units.radiansToDegrees(getIntakePosition()));
    }

    @Override
    public void simulationPeriodic(){
        IntakeSim.setInput(IntakeMotor.getMotorVoltage().getValueAsDouble());
        IntakeSim.update(0.02);

        IntakeSimEncoder.setRawRotorPosition(Units.radiansToRotations(IntakeSim.getAngleRads()*IntakeConstants.gear_ratio));
    }
}
