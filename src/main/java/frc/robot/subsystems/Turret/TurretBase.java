package frc.robot.subsystems.Turret;

<<<<<<< Updated upstream
import java.util.random.RandomGenerator.LeapableGenerator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
=======
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IdConstants;

public class TurretBase extends SubsystemBase {
<<<<<<< Updated upstream
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
    private double calibrationOffset = 0;

    public TurretBase(){ 
        motor = new TalonFX(IdConstants.BASE_MOTOR_ID);
        position = 0;
=======
    private TalonFX motor;
    private double PositionDegrees;
    private double velocity;
    private PIDController pid = new PIDController(0.02, 0, 0);

    private double versaPlanetaryGearRatio = 5.0;
    private double turretGearRatio = 140.0 / 10.0;
    private double finalGearRatio = versaPlanetaryGearRatio * turretGearRatio;

    private final int hallEffectsChannel = 0;
    private final DigitalInput hallEffectsSensor = new DigitalInput(hallEffectsChannel);
    private boolean sensorTriggered;
    private boolean motorCalibrated;

    public TurretBase() {
        motor = new TalonFX(IdConstants.Base_Motor_ID);
        PositionDegrees = 0;
>>>>>>> Stashed changes
        velocity = 0;
        pid.setTolerance(Units.degreesToRadians(2));
        sensorTriggered = false;
        motorCalibrated = false;
    }

<<<<<<< Updated upstream
    public double getPosition(){
        return position/gearRatio;
    }

    public void spinTo(double setPoint){
        pid.reset();
        pid.setSetpoint(Units.degreesToRadians(setPoint * gearRatio));
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

    public void calibrate(){
        if(sensorTriggered){
            if(velocity > 0){
                position = 0 - calibrationOffset;
            }
            else{
                position = 360 + calibrationOffset;
            }
        }
    }

    @Override
    public void periodic(){
        if(!isMotorCalibrated()){
            calibrate();
        }
        position = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
        velocity = Units.rotationsPerMinuteToRadiansPerSecond(motor.getVelocity().getValueAsDouble() * 60);
        motor.set(pid.calculate(Units.degreesToRadians(getPosition())));
        sensorTriggered = sensor.get();
=======
    public boolean isSensorTriggered() {
        return sensorTriggered;
    }

    public boolean isMotorCalibrated() {
        return motorCalibrated;
    }

    public void calibrate() {
        if (!isSensorTriggered()) {
            motor.set(0.05);
        } else {
            motor.stopMotor();
            motorCalibrated = true;
            PositionDegrees = 0;
        }
    }
    public double getPosition() {
        return PositionDegrees / finalGearRatio;
    }
    
    public double getVelocity() {
        return velocity / finalGearRatio;
    }

    public void setSetPoint(double setpoint) {
        pid.reset();
        pid.setSetpoint(Units.degreesToRadians(setpoint * finalGearRatio));
    }

    public boolean atSetPoint() {
        return pid.atSetpoint();
    }

    @Override
    public void periodic() {
        if(!isMotorCalibrated()) {
            calibrate();
        }
        velocity = Units.rotationsPerMinuteToRadiansPerSecond(motor.getVelocity().getValueAsDouble() * 60);
        sensorTriggered = hallEffectsSensor.get();
        PositionDegrees = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
        motor.set(pid.calculate(Units.degreesToRadians(getPosition())));
>>>>>>> Stashed changes
    }
}

