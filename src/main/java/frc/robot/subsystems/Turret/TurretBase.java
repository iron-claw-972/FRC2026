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
    public TurretBase(){
        motor = new TalonFX(IdConstants.BASE_MOTOR_ID);
        position = 0;
        velocity = 0;
        pid.setTolerance(0.2);
        sensorTriggered = false;
        motorCalibrated = false;
    }
    public double getPosition(){
        return position;
    }
    public void spinTo(double setPoint){
        pid.reset();
        pid.setSetpoint(setPoint);
    }
    public boolean atSetPoint(){
        return pid.atSetpoint();
    }
    public double getVelocity(){
        return velocity;
    }
    public boolean isSensorTriggered(){
        return sensorTriggered;
    }
    public boolean isMotorCalibrated(){
        return motorCalibrated;
    }
    public void calibrate(){
        if(!sensorTriggered){
            motor.set(0.02);
        }
        else{
            motor.stopMotor();
            motorCalibrated = true;
        }
    }
    @Override
    public void periodic(){
        if(!motorCalibrated){
            calibrate();
        }
        position = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
        velocity = motor.getVelocity().getValueAsDouble();
        pid.calculate(Units.degreesToRadians(getPosition()));
        sensorTriggered = sensor.get();
    }
}

