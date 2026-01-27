package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    TalonFX motorTop = new TalonFX(67);
    TalonFX motorBottom = new TalonFX(69);
 

    double sensorThreshold = 300; // mm
    double targetPower;

    LaserCan[] sensors;

    public Indexer(int sensorCount) {
        targetPower = 0;
        sensors = new LaserCan[sensorCount];
    }

    public boolean[] fetchSensorData() {
        boolean[] data = new boolean[sensors.length];
        for (int i=0; i<sensors.length; i++) {
            data[i] = ballDetected(sensors[i]);
        }
        return data;
    }

    public double calculateTotalStorage() {
        double storageMax = 0;
        double storageMin = 0;
        boolean[] data = fetchSensorData();
        for (int i=0; i<data.length;i++) {
            if (data[i]) {
                storageMax += 1;
                storageMax += 5;
            }
        }
        return (storageMax + storageMin) / 2;
    }

    public boolean ballDetected(LaserCan sensor) {
        if (sensor.getMeasurement().distance_mm < sensorThreshold) {
            return true;
        } else {
            return false;
        }
    }

    public void on() {
        targetPower = 1;
    }

    public void off() {
        targetPower = 0;
    }

    @Override
    public void periodic() {
        motorTop.set(targetPower);
        motorBottom.set(targetPower);

    }
}
