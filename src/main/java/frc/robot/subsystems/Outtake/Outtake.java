package frc.robot.subsystems.Outtake;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class Outtake extends SubsystemBase {

    private SparkFlex motor = new SparkFlex(IdConstants.OUTTAKE_MOTOR, MotorType.kBrushless);
    private double power;

    /** color sensor (measures coral based on proximity) - detects before rollers */
    private final ColorSensorV3 colorSensor = new ColorSensorV3(null); // change to proper channel
    /** beam break sensor - detects after rollers */
    private DigitalInput digitalInputEjecting = new DigitalInput(0); // change to proper channel

    public Outtake() {
        motor.configure(new SparkFlexConfig().inverted(true).idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);;
    }
}