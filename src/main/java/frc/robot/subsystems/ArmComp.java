import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.vision.ReturnData;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IdConstants;

public class ArmComp extends ArmBase {
 
    private TalonFX motor = new TalonFX(IdConstants.ARM_MOTOR);
    private PIDController pid = new PIDController(0.1, 0, 0);

    private double setpoint = ArmConstants.START_ANGLE;

    // TODO: fix gear ratio
    double gearRatio = 27.5;

    public ArmComp(){
        // Tell the PID object what the setpoint is
        // Note: at power on, the encoder will be zero
        // Ideally, the PID target value should also be zero
        // because if we were to call getAngle() right now, it would be zero rather than START_ANGLE.
        setSetpoint(ArmConstants.START_ANGLE);
    }

    @Override
    public void periodic(){
        //Obtain the motor position  
        double position = getAngle();
        // PID calculation
        double power = pid.calculate(Units.degreesToRadians(position));
        motor.set(power); 
    }

    /**
     * Set the target position (angle) of the arm.
     * @param setpoint The target angle in degrees
     */
    @Override
    public void setSetpoint(double setpoint){
        // Tell the PID object what the setpoint is
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