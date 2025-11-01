package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class ArmCompSoftPID extends ArmBase {

    private TalonFX motor = new TalonFX(IdConstants.ARM_MOTOR);
    private final DCMotor simMotor = DCMotor.getKrakenX60(1);
    private PIDController pid = new PIDController(0.1, 0, 0);
    private SingleJointedArmSim armSim;
    private TalonFXSimState encoderSim = motor.getSimState(); 

    double gearRatio = ArmConstants.GEAR_RATIO;


    public ArmCompSoftPID(){
        System.out.println("start construction");
        pid.setTolerance(Units.degreesToRadians(ArmConstants.TOLERANCE));

        if (RobotBase.isSimulation()) {
            System.out.println("is simulation");
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
        }
        System.out.println("Before pid tuner");
        // Puts the PID tuner
        SmartDashboard.putData("PID", pid);
        System.out.println("before motor set position");
        motor.setPosition(0);
        setSetpoint(ArmConstants.START_ANGLE);

        System.out.println("end of construction");
        
    }


    public void periodic(){
        double angle = getAngle();
        double powerPID = pid.calculate(Units.degreesToRadians(angle));
        motor.set(powerPID);

        displayPosition(angle);

        System.out.println("periodic");
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

     @Override
     public void setSetpoint(double setpoint) {
    
         // Tell the PID object what thsete setpoint is
         // PID is using radians
        pid.reset();
        pid.setSetpoint(Units.degreesToRadians(setpoint));
     } 
    
     @Override
    public double getAngle() {
        // when the encoder reads zero, we are at START_ANGLE degrees
        return ArmConstants.START_ANGLE
                + (Units.rotationsToDegrees(motor.getPosition().getValueAsDouble())) / gearRatio;
    }


    
}
