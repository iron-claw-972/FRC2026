package frc.robot.subsystems.Turret;

import java.util.random.RandomGenerator.LeapableGenerator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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
    private double calibrationOffset = 0;

    public TurretBase(){ 
        motor = new TalonFX(IdConstants.BASE_MOTOR_ID);
        position = 0;
        velocity = 0;
        pid.setTolerance(Units.degreesToRadians(2));
        sensorTriggered = false;
        motorCalibrated = false;
    }

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
    }
}

