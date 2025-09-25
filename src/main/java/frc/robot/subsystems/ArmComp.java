package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.vision.ReturnData;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class ArmComp extends ArmBase {

    private static final DCMotor simMotor = DCMotor.getKrakenX60(1);
    private TalonFX motor = new TalonFX(IdConstants.ARM_MOTOR);
    private PIDController pid = new PIDController(0.1, 0, 0);
    private TalonFXSimState encoderSim = motor.getSimState(); 

    private SingleJointedArmSim armSim;

    // sim? motor idk which one to use
    // simulation Objects
    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("arm", 25, ArmConstants.START_ANGLE));

    // TODO: fix gear ratio
    double gearRatio = 14;

    public ArmComp() {
        // tell the PID object the tolerance
        pid.setTolerance(Units.degreesToRadians(ArmConstants.TOLERANCE));

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
            0.1, 
            0.3,
            Units.degreesToRadians(-110), 
            Units.degreesToRadians(360),
            false,
            Units.degreesToRadians(ArmConstants.START_ANGLE));
            
        // Puts the mechanism on the smartdashboard
        SmartDashboard.putData("arm", mechanism2d);
        // Puts the PID tuner
        SmartDashboard.putData("PID", pid);
        
        // InsantCommands to set the different setpoints
        SmartDashboard.putData("Set 90 degrees", new InstantCommand(() -> setSetpoint(90)));
        SmartDashboard.putData("Set 180 degrees", new InstantCommand(() -> setSetpoint(180)));
        SmartDashboard.putData("Set 0 degrees", new InstantCommand(() -> setSetpoint(0)));
        SmartDashboard.putData("Set 67 degrees", new InstantCommand(() -> setSetpoint(67)));
    }

    @Override
    public void periodic() {
        // Obtain the motor position
        double position = getAngle();
        // PID calculation
        double power = pid.calculate(Units.degreesToRadians(position));

        // set motor power to the result of the PID calculation
        motor.set(power);
        // display the current position of the arm
        ligament2d.setAngle(position);
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
        pid.setSetpoint(Units.degreesToRadians(setpoint));
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
        // return (Math.abs(Units.radiansToDegrees(pid.getSetpoint()) - getAngle()) < ArmConstants.TOLERANCE);
        return pid.atSetpoint();
    }

    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public boolean canMoveElevator() {
        return true;
    }

}