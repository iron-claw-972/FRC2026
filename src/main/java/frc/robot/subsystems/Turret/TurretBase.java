package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class TurretBase extends SubsystemBase {
    private TalonFX motor; 
    private final DigitalInput sensor = new DigitalInput(0); 
    double power;
    /** direction the turet is pointing in degrees */
    private double position;
    /** how fast the turret is moving in radians per second */
    private double velocity;
    private PIDController pid = new PIDController(0.002, 0, 0);
    private boolean sensorTriggered;

    private boolean motorCalibrated;


    private double versaPlanetaryGearRatio = 5.0;
    /** 140 teeth divided by 10 teeth */
    private double turretGearRatio = 140.0/10.0;
    private final double gearRatio = versaPlanetaryGearRatio * turretGearRatio;
    private double calibrationOffset = 0;
    
    private SingleJointedArmSim turretSim;
    private static final DCMotor simMotor = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;
    
    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50,50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("turret", 25, 0));

    public TurretBase() { 
        motor = new TalonFX(IdConstants.BASE_MOTOR_ID);
        encoderSim = motor.getSimState();
        pid.setTolerance(Units.degreesToRadians(2));
        sensorTriggered = false;
        motorCalibrated = false;

        turretSim = new SingleJointedArmSim(
            simMotor,
            turretGearRatio,
            0.1,
            0.3,
            0,
            Units.degreesToRadians(360),
            false,
            Units.degreesToRadians(-360)
        );
            
        SmartDashboard.putData("turret", mechanism2d);
        SmartDashboard.putData("PID", pid);
        SmartDashboard.putBoolean("Calibrated", motorCalibrated);

        
        SmartDashboard.putData("Set 90 degrees", new InstantCommand(() -> spinTo(90)));
        SmartDashboard.putData("Set 180 degrees", new InstantCommand(() -> spinTo(180)));
        SmartDashboard.putData("Set 0 degrees", new InstantCommand(() -> spinTo(0)));
        SmartDashboard.putData("Set 270 degrees", new InstantCommand(() -> spinTo(270)));

        pid.enableContinuousInput(-Math.PI, Math.PI);

    }

    /** position in degrees of the turret not the turret motor */
    public double getPosition(){
        return position;
    }

    /** 
     * set the target angle
     * @param setPoint angle in degrees
     */
    public void spinTo(double setPoint){
        pid.reset();
        pid.setSetpoint(Units.degreesToRadians(setPoint));
    }

    public boolean atSetPoint(){
        return pid.atSetpoint();
    }

    public double getVelocity(){
        return velocity/gearRatio;
    }

    public boolean isSensorTriggered(){
        return sensorTriggered;
    }

    public boolean isMotorCalibrated(){
        return motorCalibrated;
    }

    public void calibrate() {
        motor.set(0.02);
        sensorTriggered = sensor.get();
        if(isSensorTriggered()) {
            if(velocity > 0) {
                position = 0 - calibrationOffset;
            } else {
                position = 0 + calibrationOffset;
            }
            motorCalibrated = true;
        }
    }

    @Override
    public void periodic() {
        if(!isMotorCalibrated()) {
            calibrate();
        } else {
            position = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
            velocity = Units.rotationsPerMinuteToRadiansPerSecond(motor.getVelocity().getValueAsDouble() * 60);
            /** feeding the PID radians */
            power = pid.calculate(Units.degreesToRadians(getPosition()));
            motor.set(power);
            System.out.println(power);
            System.out.println("Check " + motor.get());
            sensorTriggered = sensor.get();
    
            ligament2d.setAngle(position);
        }
    }

    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        double voltsMotor = power * 12;
        System.out.println("Motor " + voltsMotor);
        turretSim.setInputVoltage(voltsMotor);

        turretSim.update(Constants.LOOP_TIME);

        double simAngle = turretSim.getAngleRads();
        double simRotations = Units.radiansToRotations(simAngle);
        double motorRotations = simRotations * gearRatio;
        System.out.println("Rotations " + motorRotations);
        
        encoderSim.setRawRotorPosition(motorRotations);
    }
}

