package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.vision.ReturnData;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class ArmComp extends ArmBase {
 
    private static final DCMotor TalonFX = null;
    private TalonFX motor = new TalonFX(IdConstants.ARM_MOTOR);
    private PIDController pid = new PIDController(0.1, 0, 0);

    private double setpoint = ArmConstants.START_ANGLE;

    private SingleJointedArmSim armSim;

    // sim? motor idk which one to use
    DCMotor dcMotor;
    // simulation Objects
    Mechanism2d mechanism2d;
    MechanismLigament2d ligament2d;

    // TODO: fix gear ratio
    double gearRatio = 27.5;

    public ArmComp(){
        // Tell the PID object what the setpoint is
        // Note: at power on, the encoder will be zero
        // Ideally, the PID target value should also be zero
        // because if we were to call getAngle() right now, it would be zero rather than START_ANGLE.
        setSetpoint(ArmConstants.START_ANGLE);
        // simulation Arm
        armSim = new SingleJointedArmSim(dcMotor, gearRatio, 
        0.1, 0.05, 
        Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, 
        false, ArmConstants.START_ANGLE, null);
        mechanism2d = new Mechanism2d(100, 100);
        ligament2d = new MechanismLigament2d("Arm try", 25, ArmConstants.START_ANGLE);
        mechanism2d.getRoot("pivot", 50, 50).append(ligament2d);
        SmartDashboard.putData("Arm Display", mechanism2d);

    }

    @Override
    public void periodic(){
        //Obtain the motor position  
        double position = getAngle();
        // PID calculation
        double power = pid.calculate(Units.degreesToRadians(position));
        motor.set(power); 
    }

    public void simulationPeriodic(){
        double position = getAngle();
        armSim.update(Constants.LOOP_TIME);
        ligament2d.setAngle(position);
        setSetpoint(pid.calculate(Units.degreesToRadians(position)));
    }

    /**
     * Set the target position (angle) of the arm.
     * @param setpoint The target angle in degrees
     */
    @Override
    public void setSetpoint(double setpoint){
        // Tell the PID object what thsete setpoint is
        // PID is using radians
        pid.setSetpoint(Units.degreesToRadians(setpoint * gearRatio));

        this.setpoint=setpoint;
    }

    /** Gets the arm angle in degrees */
    @Override
    public double getAngle(){
        // when the encoder reads zero, we are at START_ANGLE degrees
        return ArmConstants.START_ANGLE + (Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()))/gearRatio;
    }

    @Override
    public boolean atSetpoint(){
        return (Math.abs(setpoint - getAngle()) < ArmConstants.TOLERANCE);
    }

    public double getAppliedVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public boolean canMoveElevator(){
        return true;
    }

    
    


}