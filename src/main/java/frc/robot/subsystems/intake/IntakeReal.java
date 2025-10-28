package frc.robot.subsystems.intake;
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

import frc.robot.constants.HoodConstants;
import frc.robot.constants.swerve.DriveConstants;

public class IntakeReal extends IntakeBase{
    private final int flywheel_motor_id = 2;
    private final int intake_motor_id = 3;

    TalonFX IntakeMotor;
    TalonFX FlywheelMotor;

    PIDController IntakePID;

    private final double kP=0.02;
    private final double kI=0.0;
    private final double kD=0.0;

    private final double max_velocity = 1.0;
    private final double max_acceleration = 2.0;

    private final double gear_ratio=1.0;

    private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(max_velocity, max_acceleration));         

    ArmFeedforward IntakeFeedforward;

    private final double gravity = 9.81;
    private final double arm_mass = 2.46;
    private final double arm_length = HoodConstants.LENGTH;

    private double globalIntakePower = 0.0d;

    private TalonFXSimState IntakeSimEncoder;

    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("IntakeMotor", 25, 0));

    DCMotor sim_motor = DCMotor.getKrakenX60(1);

    private SingleJointedArmSim IntakeSim = 
    new SingleJointedArmSim(
        DCMotor.getKrakenX60(1), 
        gear_ratio, 
        SingleJointedArmSim.estimateMOI(arm_length, arm_mass), 
        arm_length, 
        0, 
        Units.degreesToRadians(360), 
        true, 
        0);    


    public IntakeReal(){
        IntakeMotor = new TalonFX(intake_motor_id);
        FlywheelMotor = new TalonFX(flywheel_motor_id);

        IntakeSimEncoder = IntakeMotor.getSimState();

        IntakePID = new PIDController(kP, kI, kD);

        //m_controller.enableContinuousInput(0, Units.degreesToRadians(360));

        m_controller.setTolerance(0.01);

        IntakeFeedforward = new ArmFeedforward
            (0.0,
            gravity * arm_length/2.0 * arm_mass
                    / gear_ratio * sim_motor.rOhms / sim_motor.KtNMPerAmp / 12,
            0);

        SmartDashboard.putData("IntakePID", IntakePID);
        SmartDashboard.putData("Motion Magic PID Controller", m_controller);
        SmartDashboard.putData("Intake", mechanism2d);

        SmartDashboard.putData("Set 90 degrees", new InstantCommand(() -> setIntakeSetpoint(90)));
        SmartDashboard.putData("Set 180 degrees", new InstantCommand(() -> setIntakeSetpoint(180)));
        SmartDashboard.putData("Set 0 degrees", new InstantCommand(() -> setIntakeSetpoint(0)));
        SmartDashboard.putData("Set 270 degrees", new InstantCommand(() -> setIntakeSetpoint(270)));
    }

    public void setIntakeSetpoint(double position){
        IntakePID.reset();
        IntakePID.setSetpoint(Units.degreesToRadians(position/gear_ratio));

        m_controller.setGoal(Units.degreesToRadians(position/gear_ratio));
    }
    
    public double getIntakePosition(){
        return Units.rotationsToRadians(IntakeMotor.getPosition().getValueAsDouble());
    }

    public double getIntakeVelocity(){
        return Units.rotationsPerMinuteToRadiansPerSecond(IntakeMotor.getVelocity().getValueAsDouble() * 60);
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
    public void periodic(){
        globalIntakePower = /*IntakePID.calculate(getIntakePosition()) PID */ m_controller.calculate(getIntakePosition()) + 
        IntakeFeedforward.calculate(getIntakePosition(), 0); // Feed Forward
        //IntakeFeedforward.calculate(getIntakePosition(), 0);
        // IntakePower.set(globalIntakePower);

    }

    @Override
    public void simulationPeriodic(){
        IntakeSim.setInput(globalIntakePower * 12);
        IntakeSim.update(0.02);

        IntakeSimEncoder.setRawRotorPosition(Units.radiansToRotations(IntakeSim.getAngleRads()));

        ligament2d.setAngle(Units.radiansToDegrees(getIntakePosition()));
    }
}
