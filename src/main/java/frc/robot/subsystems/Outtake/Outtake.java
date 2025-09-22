package frc.robot.subsystems.Outtake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class Outtake extends SubsystemBase {

    private TalonFX motor = new TalonFX(IdConstants.OUTTAKE_MOTOR);
    private double power;

    /** color sensor (measures coral based on proximity) - detects before rollers */
    private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);


    public Outtake() {
        motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Brake));
    }


    public double getMotorSpeed() {
        return power;
    }
    @Override
    public void periodic() {
        motor.set(power);
    }

    public void setMotor(double power) {
        this.power = power;
    }

    public void outtake(){
        setMotor(-0.3);
    }

    public int getProximity() {
        int proximity = colorSensor.getProximity();
        if (proximity > 0) {
            return proximity;
        } else {
            I2CJNI.i2CClose(1);
            colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
            return colorSensor.getProximity();
        }
    }

    public void reverse(){
        setMotor(0.3);
    }

    public boolean coralLoaded() {
        return getProximity() > 2000;
    }

    public boolean coralEjecting() {
        return coralLoaded();
    }
}