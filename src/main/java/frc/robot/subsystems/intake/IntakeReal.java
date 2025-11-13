package frc.robot.subsystems.intake;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.constants.IntakeConstants;
import frc.robot.constants.swerve.DriveConstants;

public class IntakeReal extends IntakeBase{
    TalonFX IntakeMotor;
    TalonFX FlywheelMotor;

    PIDController IntakePID;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);


    private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          IntakeConstants.kP,
          IntakeConstants.kI,
          IntakeConstants.kD,
          new TrapezoidProfile.Constraints(IntakeConstants.max_velocity, IntakeConstants.max_acceleration));

    ArmFeedforward IntakeFeedforward;

    private double globalIntakePower = 0.0d;

    private TalonFXSimState IntakeSimEncoder;

    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("IntakeMotor", 25, 0));

    DCMotor sim_motor = DCMotor.getKrakenX60(1);

    private SingleJointedArmSim IntakeSim = 
    new SingleJointedArmSim(
        DCMotor.getKrakenX60(1), 
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

        //m_controller.enableContinuousInput(0, Units.degreesToRadians(360));

        m_controller.setTolerance(0.01);

        IntakeFeedforward = new ArmFeedforward
            (0.0,
            IntakeConstants.gravity * IntakeConstants.center_of_mass * (IntakeConstants.arm_mass)
                    / IntakeConstants.gear_ratio * sim_motor.rOhms / sim_motor.KtNMPerAmp / IntakeConstants.robot_voltage,
            0);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = 0.1; // Static friction compensation (should be >0 if friction exists)
        config.Slot0.kG = IntakeConstants.arm_mass * IntakeConstants.center_of_mass * 9.8 / IntakeConstants.gear_ratio; // Gravity compensation
        config.Slot0.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        config.Slot0.kA = 0; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        config.Slot0.kP = Units.radiansToRotations(5.0 * 12); // If position error is 2.5 rotations, apply 12V (0.5 * 2.5 * 12V)
        config.Slot0.kI = Units.radiansToRotations(0.00); // Integral term (usually left at 0 for MotionMagic)
        config.Slot0.kD = Units.radiansToRotations(0.00 * 12); // Derivative term (used to dampen oscillations)

        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Units.radiansToRotations(IntakeConstants.max_velocity * IntakeConstants.gear_ratio);
        motionMagicConfigs.MotionMagicAcceleration = Units.radiansToRotations(IntakeConstants.max_acceleration * IntakeConstants.gear_ratio);
        

        IntakeMotor.getConfigurator().apply(config);

        SmartDashboard.putData("IntakePID", IntakePID);
        SmartDashboard.putData("Motion Magic PID Controller", m_controller);
        SmartDashboard.putData("Intake", mechanism2d);

        SmartDashboard.putData("Set 90 degrees", new InstantCommand(() -> setIntakeSetpoint(90)));
        SmartDashboard.putData("Set 180 degrees", new InstantCommand(() -> setIntakeSetpoint(180)));
        SmartDashboard.putData("Set 0 degrees", new InstantCommand(() -> setIntakeSetpoint(0)));
        SmartDashboard.putData("Set 270 degrees", new InstantCommand(() -> setIntakeSetpoint(270)));
    }

    public void setIntakeSetpoint(double position){
        IntakeMotor.setControl(m_request.withPosition(position));
    }
    
    public double getIntakePosition(){
        return Units.rotationsToRadians(IntakeMotor.getPosition().getValueAsDouble());
    }

    public double getIntakeVelocity(){
        return Units.rotationsPerMinuteToRadiansPerSecond(IntakeMotor.getVelocity().getValueAsDouble() * IntakeConstants.gear_ratio * 60);
    }

    public boolean IntakeAtSetpoint(){
        //return IntakePID.atSetpoint();
        return m_controller.atSetpoint();
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

    @Override
    public void periodic() {
        globalIntakePower =  m_controller.calculate(getIntakePosition()) + 
        IntakeFeedforward.calculate(getIntakePosition(), 0);
        //IntakeFeedforward.calculate(getIntakePosition(), 0);
        // IntakePower.set(globalIntakePower);

    }

    @Override
    public void simulationPeriodic(){
        IntakeSim.setInput(globalIntakePower * IntakeConstants.robot_voltage);
        IntakeSim.update(0.02);

        IntakeSimEncoder.setRawRotorPosition(Units.radiansToRotations(IntakeSim.getAngleRads()));

        ligament2d.setAngle(Units.radiansToDegrees(getIntakePosition()));
    }
}
