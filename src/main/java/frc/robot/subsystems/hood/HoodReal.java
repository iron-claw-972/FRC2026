package frc.robot.subsystems.hood;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;

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
import frc.robot.constants.Constants;
import frc.robot.constants.HoodConstants;

public class HoodReal extends HoodBase{
    double kP = 0.01;
    double kI = 0;
    double kD = 0.5;
    PIDController hoodPid = new PIDController(kP, kI, kD);

    private TalonFX motor;

    private double position;
    private double velocity;

    private final int motorId = 1;

    private double hoodGearRatio = 67.0/67.0;

    SingleJointedArmSim hood_sim;
    Mechanism2d mechanism2d;
    MechanismLigament2d ligament2d;

    private TalonFXSimState encoderSim;
    private double power;

    public HoodReal(){
        //motor = new TalonFX(motorId);

        hood_sim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1), 
            hoodGearRatio,
            HoodConstants.MOI,
            HoodConstants.LENGTH,
            0,
            Units.degreesToRadians(360),
            false,
            Units.degreesToRadians(HoodConstants.START_ANGLE)
        );

        motor = new TalonFX(motorId);

        encoderSim = motor.getSimState();

        mechanism2d = new Mechanism2d(100, 100);
        ligament2d = new MechanismLigament2d("hood_ligament", 25, 0);

        mechanism2d.getRoot("pivot", 50, 50).append(ligament2d);

        SmartDashboard.putData("hood", mechanism2d);
        SmartDashboard.putData("PID", hoodPid);
        
        SmartDashboard.putData("Set 90 degrees", new InstantCommand(() -> setSetpoint(90)));
        SmartDashboard.putData("Set 180 degrees", new InstantCommand(() -> setSetpoint(180)));
        SmartDashboard.putData("Set 0 degrees", new InstantCommand(() -> setSetpoint(0)));
        SmartDashboard.putData("Set 270 degrees", new InstantCommand(() -> setSetpoint(270)));
    }

    public void setSetpoint(double setpoint){
       hoodPid.reset();
       hoodPid.setSetpoint(Units.degreesToRadians(setpoint));
    }
    
    public double getPosition(){
        return position;
    }

    public double getVelocity(){
        return velocity/hoodGearRatio;
    }

    public boolean atSetpoint(){
        return hoodPid.atSetpoint();
    }

    public void periodic(){
        // Uncomment when not in simulation
       //position = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
       velocity = motor.getVelocity().getValueAsDouble();

       power = hoodPid.calculate(getPosition());
       motor.set(power);
       ligament2d.setAngle(Units.radiansToDegrees(getPosition()));
    }

    @Override
    public void simulationPeriodic(){
        //System.out.println("Setpoint" + hoodPid.getSetpoint() + "@" + Timer.getFPGATimestamp());

        hood_sim.setInput(power * RobotController.getBatteryVoltage());

        hood_sim.update(Constants.LOOP_TIME);

        encoderSim.setRawRotorPosition(Units.radiansToRotations(hood_sim.getAngleRads()));
        position = hood_sim.getAngleRads();
    }
}