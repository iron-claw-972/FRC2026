package frc.robot;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

public class CANRangeSensorTest extends TimedRobot {

    private static final int SENSOR_ID = 0;

    private CANrange rangeSensor;
    private boolean sensorConnected = false;

    @Override
    public void robotInit() {
        try {
            rangeSensor = new CANrange(SENSOR_ID, Constants.CANIVORE_CAN);
            sensorConnected = true;
        } catch (Exception e) {
            sensorConnected = false;
        }
    }

    @Override
    public void robotPeriodic() {
        if (!sensorConnected) {
            SmartDashboard.putString("CANrange Status", "NOT CONNECTED");
            SmartDashboard.putNumber("Distance (m)", -1);
            SmartDashboard.putNumber("Signal Strength", -1);
            return;
        }

        try {
            double distanceMeters = rangeSensor.getDistance().getValueAsDouble();
            double signalStrength = rangeSensor.getSignalStrength().getValueAsDouble();

            SmartDashboard.putString("CANrange Status", "CONNECTED");
            SmartDashboard.putNumber("Distance (cm)", distanceMeters * 100);
            SmartDashboard.putNumber("Signal Strength %", signalStrength * 100);

        } catch (Exception e) {
            SmartDashboard.putString("CANrange Status", "ERROR");
        }
    }
}
