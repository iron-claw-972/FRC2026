package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;

public class BreakerModelUtil {
    private HashMap<Double, Double> currents = new HashMap<>();
    public final Timer timer = new Timer();

    public BreakerModelUtil() {
        timer.reset();
    }

    public void addCurrent(double current) {
        currents.put(timer.get(), current);
    }

    public double average(double time) {
        double sum = 0.0;
        int numberOfRegisteredIntervals = 0;
        for (Map.Entry<Double, Double> entry : currents.entrySet()) {
            // if time of this interval is after the specified start time
            if (entry.getKey() > time) {
                sum += entry.getValue();
                numberOfRegisteredIntervals += 1;
            }
        }
        double avg = sum / numberOfRegisteredIntervals;

        return avg;
    }

    public double findClosestTime(double time) {
        // obtains the key closest to the specified time
        double closest = 0.0;
        for (double key : currents.keySet()) {
            if (Math.abs(key - time) < Math.abs(closest - time)) {
                closest = key;
            }
        }
        return closest;
    }

    public List<Map.Entry<Double, Double>> findInstancesAboveThreshold(double thresholdAmps) {
        List<Map.Entry<Double, Double>> instances = new ArrayList<>();
        for (Map.Entry<Double, Double> entry : currents.entrySet()) {
            if (entry.getValue() > thresholdAmps) {
                instances.add(entry);
            }
        }
        return instances;
    }

    public List<Map.Entry<Double, Double>> findInstancesBelowThreshold(double thresholdAmps) {
        List<Map.Entry<Double, Double>> instances = new ArrayList<>();
        for (Map.Entry<Double, Double> entry : currents.entrySet()) {
            if (entry.getValue() < thresholdAmps) {
                instances.add(entry);
            }
        }
        return instances;
    }

    public void clear() {
        currents.clear();
    }
}
