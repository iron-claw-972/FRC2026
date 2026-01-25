package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.util.ClimbArmSim;

public class Climb extends SubsystemBase {
    
    private double startingPosition = 0;

    //Motors
    // TODO: tune better once design is finalized
    private final PIDController pid = new PIDController(0.4, 4, 0.04);

    private TalonFX motorLeft = new TalonFX(IdConstants.CLIMB_MOTOR_LEFT);
    private TalonFX motorRight = new TalonFX(IdConstants.CLIMB_MOTOR_RIGHT);

    private final DCMotor climbGearBox = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;

    //Mechism2d display
    private final Mechanism2d simulationMechanism = new Mechanism2d(3, 3);
    private final MechanismRoot2d mechanismRoot = simulationMechanism.getRoot("Climb", 1.5, 1.5);
    private final MechanismLigament2d simLigament = mechanismRoot.append(
        new MechanismLigament2d("angle", 1, startingPosition, 4, new Color8Bit(Color.kAntiqueWhite))
    );

    private final double climbGearRatio = 54 / 10 * 60 / 18; // gear ratio of 18
    private ClimbArmSim climbSim;
    private double power;

    public Climb() {
        if (isSimulation()) {
            encoderSim = motorLeft.getSimState();
            encoderSim.setRawRotorPosition(Units.degreesToRotations(startingPosition)*climbGearRatio);

            climbSim = new ClimbArmSim(
                climbGearBox, 
                climbGearRatio, 
                0.1, 
                0.127, 
                0, //min angle 
                Units.degreesToRadians(90), //max angle
                true, 
                Units.degreesToRadians(startingPosition),
                60
            );

            climbSim.setIsClimbing(true);
        }

        pid.setIZone(1);
        pid.setSetpoint(Units.degreesToRadians(startingPosition));

        motorLeft.setPosition(Units.degreesToRotations(startingPosition)*climbGearRatio);
        motorRight.setPosition(Units.degreesToRotations(startingPosition)*climbGearRatio);

        SmartDashboard.putData("PID", pid);
        SmartDashboard.putData("Climb Display", simulationMechanism);
         SmartDashboard.putData("EXTEND", new InstantCommand(() -> extend()));
        SmartDashboard.putData("CLIMB", new InstantCommand(() -> climb()));
    }

    @Override
    public void periodic() { 
        double motorPosition = motorLeft.getPosition().getValueAsDouble();
        double currentPosition = Units.rotationsToRadians(motorPosition/climbGearRatio);

        power = pid.calculate(currentPosition);

        motorLeft.set(MathUtil.clamp(power, -1, 1));
        motorRight.set(MathUtil.clamp(-power, -1, 1)); // Invert motor

        simLigament.setAngle(Units.radiansToDegrees(currentPosition));

        SmartDashboard.putNumber("Climb Position", getAngle());

        SmartDashboard.putNumber("Encoder Position", motorLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Motor Velocity", motorLeft.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        climbSim.setInput(power * Constants.ROBOT_VOLTAGE);
        climbSim.update(Constants.LOOP_TIME);

        double climbRotations = Units.radiansToRotations(climbSim.getAngleRads());
        encoderSim.setRawRotorPosition(climbRotations * climbGearRatio);
    }

    /**
     * Sets the motor to an angle from 0-90 deg
     * @param angle in degrees
     */
    public void setAngle(double angle) {
        pid.reset();
        pid.setSetpoint(Units.degreesToRadians(angle));
    }

    /**
     * Gets the current position of the motor in degrees
     * @return The angle in degrees
     */
    public double getAngle() {
        return Units.rotationsToDegrees(motorLeft.getPosition().getValueAsDouble() / climbGearRatio);
    }

    /**
     * Turns the motor to 90 degrees (extended positiion)
     */
    public void extend(){
        double extendAngle = 90;
        setAngle(extendAngle);
    }

    /**
     * Turns the motor to 0 degrees (climb position)
     */
    public void climb(){
        setAngle(startingPosition);
    }

    public boolean isSimulation(){
        return RobotBase.isSimulation();
    }
}