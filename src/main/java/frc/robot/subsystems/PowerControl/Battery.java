package frc.robot.subsystems.PowerControl;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Battery extends SubsystemBase {
    private double voltage;
    public Battery() {
        updateVoltageFromBattery();
    }

    private void updateVoltageFromBattery() { 
        voltage = RobotController.getBatteryVoltage();
    }
    public double getVoltage() { return voltage; }

    public double voltsTillBrownOut() { return voltage - RobotController.getBrownoutVoltage(); }
    public double toBrownOut() { 
        // percent of volts we've got left over what we had to start with
        return voltsTillBrownOut() / (BatteryConstants.MAX_STARTING_VOLTS - RobotController.getBrownoutVoltage());
    }

    @Override
    public void periodic() {
        updateVoltageFromBattery();
    }
}
