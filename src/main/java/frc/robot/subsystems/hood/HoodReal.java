import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class HoodReal extends HoodBase{
    double kP = 0.01;
    double kI = 0;
    double kD = 0;
    PIDController hoodPid = new PIDController(kP, kI, kD);

    private TalonFX motor;

    private double position;

    private final int motorId = -1;

    SingleJointedArmSim hood_sim;

    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50,50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("hood", 25, 0));

    private TalonFXSimState encoderSim;

    public HoodReal(){
        motor = new TalonFX(motorId);

        encoderSim = motor.getSimState();

        hood_sim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 
            kD, 
            kD, 
            motorId, 
            kP, 
            kI, 
            true, 
            kD, 
            null
        );
    }

    public void setSetpoint(double setpoint){
       hoodPid.setSetpoint(setpoint);
    }
    
    public double getPosition(){
        return position;
    }

    public double getVelocity(){}

    public boolean atSetpoint(){
        hoodPid.atSetpoint();
    }

    public void periodic(){
       position = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());

       motor.set(hoodPid.calculate(getPosition()));
    }

    public void simulationPeriodic(){
        position = 
    }
}