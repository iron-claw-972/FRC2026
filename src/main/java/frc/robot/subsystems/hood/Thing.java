package frc.robot.subsystems.hood;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Thing extends SubsystemBase {

    private final TalonFX wheelMotor =
        new TalonFX(IdConstants.HOOD_ID, Constants.CANIVORE_SUB);

    public Thing() {
    }

    public void spin(double speed) {
        wheelMotor.set(speed);
    }

    public void stop() {
        wheelMotor.stopMotor();
    }
}