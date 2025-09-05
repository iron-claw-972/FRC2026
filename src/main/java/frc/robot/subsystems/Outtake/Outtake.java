package frc.robot.subsystems.Outtake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class Outtake extends SubsystemBase{
    private SparkFlex motor = new SparkFlex(IdConstants.OUTTAKE_MOTOR, MotorType.kBrushless);


    public Outtake() {
        motor.configure(new SparkFlexConfig(), null, null);
    }
}
