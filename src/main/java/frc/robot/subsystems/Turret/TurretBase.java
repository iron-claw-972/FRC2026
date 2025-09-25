package frc.robot.subsystems.Turret;

import java.util.random.RandomGenerator.LeapableGenerator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class TurretBase extends SubsystemBase {
    private TalonFX motor; 
    private final DigitalInput sensor = new DigitalInput(0); 
    private double position;
    private double velocity;
    private PIDController pid = new PIDController(0.02, 0, 0);
    private boolean sensorTriggered;
    private boolean motorCalibrated;
    private double versaPlanetaryGearRatio = 5.0;
    private double turretGearRatio = 140/10;
    private final double gearRatio = versaPlanetaryGearRatio * turretGearRatio;
    // TODO: Fix the way we are setting the offset
    private double calibrationOffset = 0;
    private double offset = 0; 

    private SingleJointedArmSim turretSim;
    private static final DCMotor simMotor = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim = motor.getSimState();
    
    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50,50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("turret", 25, 0));

    public TurretBase(){ 
        motor = new TalonFX(IdConstants.BASE_MOTOR_ID);
        pid.setTolerance(Units.degreesToRadians(2));
        sensorTriggered = false;
        motorCalibrated = false;
        turretSim = new SingleJointedArmSim(
            simMotor,
            gearRatio,
            0.1,
            0.3,
            0,
            Units.degreesToRadians(300),
            false,
            Units.degreesToRadians(0));
        
        SmartDashboard.putData("Turret", mechanism2d);
        SmartDashboard.putData("PID", pid);

        SmartDashboard.putData("Go to 90 degrees", new InstantCommand(() -> spinTo(90)));
        SmartDashboard.putData("Go to 270 degrees", new InstantCommand(() -> spinTo(270)));
        SmartDashboard.putData("Go to 180 degrees", new InstantCommand(() -> spinTo(180)));
        SmartDashboard.putData("Go to 67 degrees", new InstantCommand(() -> spinTo(67)));

    }

    public double getPosition(){
        return (Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / gearRatio) + offset;
    }

    public void spinTo(double setPoint){
        pid.reset();
        pid.setSetpoint(Units.degreesToRadians(setPoint * gearRatio));
    }

    public boolean atSetPoint(){
        return pid.atSetpoint();
    }

    public double getVelocity(){
        return Units.rotationsPerMinuteToRadiansPerSecond(motor.getVelocity().getValueAsDouble() * 60) / gearRatio;
    }

    public boolean isSensorTriggered(){
        return sensorTriggered;
    }

    public boolean isMotorCalibrated(){
        return motorCalibrated;
    }

    public void calibrate(){
        if(sensorTriggered){
            if(getVelocity() > 0){
                offset = 0 - calibrationOffset;
            }
            else{
                offset = 360 + calibrationOffset;
            }
        }
    }

    @Override
    public void periodic(){
        if(!isMotorCalibrated()){
            calibrate();
        }
        motor.set(pid.calculate(Units.degreesToRadians(getPosition())));
        sensorTriggered = sensor.get();

        ligament2d.setAngle(position);
    }

    public void simulationPeriodic() {
        // motor.get() * 12.0
        turretSim.setInputVoltage(motor.getMotorVoltage().getValueAsDouble());

        turretSim.update(Constants.LOOP_TIME);

        double simAngle = turretSim.getAngleRads();

        double motorRotations = Units.radiansToRotations(simAngle) * gearRatio;

        encoderSim.setRawRotorPosition(motorRotations);
    }
}

