package frc.robot.subsystems.hood;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.IntakeConstants;

public class HoodReal extends HoodBase{
    private TalonFX HoodMotor;
    DCMotor simMotor = DCMotor.getFalcon500(1);

    SingleJointedArmSim hood_sim;
    Mechanism2d mechanism2d;
    MechanismLigament2d ligament2d;

    private TalonFXSimState encoderSim;

    private final MotionMagicVoltage request = new MotionMagicVoltage(0);

    private double Setpoint = 0.0;

    private ArmFeedforward HoodFeedForward;

    public HoodReal(){

        hood_sim = new SingleJointedArmSim(
            simMotor, 
            1.0/HoodConstants.gear_ratio,
            HoodConstants.MOI,
            HoodConstants.LENGTH,
            0,
            Units.degreesToRadians(360),
            true,
            Units.degreesToRadians(HoodConstants.START_ANGLE)
        );

        HoodMotor = new TalonFX(HoodConstants.motor_id);

        HoodMotor.getConfigurator().apply(getMotionMagicConfig());

        encoderSim = HoodMotor.getSimState();

        // V=IR
        // (Torque/KtPerAmp) = I
        // Ohms is SimMotor property

        double torque = ((HoodConstants.CENTER_OF_MASS_LENGTH * HoodConstants.MASS * HoodConstants.gravity) * HoodConstants.gear_ratio);
        double current = (torque/simMotor.KtNMPerAmp);

        HoodFeedForward = new ArmFeedforward(0, 
            current*simMotor.rOhms, 
            0);

        mechanism2d = new Mechanism2d(100, 100);
        ligament2d = new MechanismLigament2d("hood_ligament", 25, 0);

        mechanism2d.getRoot("pivot", 50, 50).append(ligament2d);

        SmartDashboard.putData("hood", mechanism2d);
        
        SmartDashboard.putData("Set 90 degrees", new InstantCommand(() -> setSetpoint(Units.degreesToRadians(90))));
        SmartDashboard.putData("Set 180 degrees", new InstantCommand(() -> setSetpoint(Units.degreesToRadians(180))));
        SmartDashboard.putData("Set 0 degrees", new InstantCommand(() -> setSetpoint(Units.degreesToRadians(0))));
        SmartDashboard.putData("Set 270 degrees", new InstantCommand(() -> setSetpoint(Units.degreesToRadians(270))));
    }

    public TalonFXConfiguration getMotionMagicConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kS = 0.1; // Static friction compensation (should be >0 if friction exists)
        config.Slot0.kG = (HoodConstants.MASS * HoodConstants.CENTER_OF_MASS_LENGTH * 9.8) / HoodConstants.gear_ratio; // Gravity compensation
        config.Slot0.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        config.Slot0.kA = 0; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        config.Slot0.kP = Units.radiansToRotations(2.0 * 12); // If position error is 2.5 rotations, apply 12V (0.5 * 2.5 * 12V)
        config.Slot0.kI = Units.radiansToRotations(0.0); // Integral term (usually left at 0 for MotionMagic)
        config.Slot0.kD = Units.radiansToRotations(1.0 * 12); // Derivative term (used to dampen oscillations)

        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Units.radiansToRotations(HoodConstants.MAX_VELOCITY) * HoodConstants.gear_ratio;
        motionMagicConfigs.MotionMagicAcceleration = Units.radiansToRotations(HoodConstants.MAX_ACCELERATION) * HoodConstants.gear_ratio;

        return config;
    }

    public void setSetpoint(double setpoint){
       Setpoint = Units.radiansToRotations(setpoint)/HoodConstants.gear_ratio;

       HoodMotor.setControl(request.withPosition(Setpoint).withFeedForward(HoodFeedForward.calculate(getHoodPosition(), 0)));
    }
    
    public double getHoodPosition(){
        return Units.rotationsToRadians(HoodMotor.getPosition().getValueAsDouble() * HoodConstants.gear_ratio);
    }

    public double getVelocity(){
        // (rotations/second) * (2pi/1 rotation)
        return Units.rotationsToRadians(HoodMotor.getVelocity().getValueAsDouble());
    }

    public boolean atSetpoint(){
        return Math.abs((request.Position * HoodConstants.gear_ratio) - getHoodPosition()) < 0.01;
    }

    public void periodic(){
       ligament2d.setAngle(Units.radiansToDegrees(getHoodPosition()));

       HoodMotor.setControl(request.withPosition(Setpoint).withFeedForward(HoodFeedForward.calculate(getHoodPosition(), 0)));

       System.out.println(Setpoint * HoodConstants.gear_ratio);
    }

    public void ShootHood(double distance, double height){
        double theta = Math.atan( (2*height)/distance );
        double velocity = Math.sqrt(2*HoodConstants.gravity*height)/Math.sin(theta);

        this.setSetpoint( Math.atan( (2*height)/distance ));
    }

    @Override
    public void simulationPeriodic(){
        hood_sim.setInput(HoodMotor.getMotorVoltage().getValueAsDouble());

        hood_sim.update(HoodConstants.LOOP_TIME);

        encoderSim.setRawRotorPosition(Units.radiansToRotations(hood_sim.getAngleRads())/HoodConstants.gear_ratio);
    }
}