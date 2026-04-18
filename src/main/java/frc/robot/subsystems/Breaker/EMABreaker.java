package frc.robot.subsystems.Breaker;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class EMABreaker extends SubsystemBase {

    private static class Current {
        double alpha; // how much of the error we correct per loop
        double average = 0;
        double threshold;
    }

    PowerDistribution pDis = new PowerDistribution();

    private List<Current> filters = new ArrayList<>(); // contains currents with their alphas and thresholds

    public EMABreaker() {
        for (Map.Entry<Double, Double> entry : BreakerConstants.THRESHOLDS.entrySet()) {
            double tau = entry.getKey(); // sec
            double threshold = entry.getValue(); // A

            Current w = new Current(); // create a filter for the threshold
            w.threshold = threshold;
            w.alpha = 1 - Math.exp(-Constants.LOOP_TIME / tau); // 1 - e^(-0.02/1) = 0.0198, 1 - e^(-0.02/2) = 0.00995 

            filters.add(w);
        }
    }

    @Override
    public void periodic() {
        double current = getCurrentFromPowerDistribution();
        for (Current f : filters) {
            // new avg = old avg + fractionAlpha * difference
            f.average += f.alpha * (current - f.average);
        }
    }

    public double getCurrentFromPowerDistribution() {
        return pDis.getTotalCurrent(); // not using .getCurrent() and then an arguement for the port you can get just one port
    }

    public boolean isOverCurrent() {
        for (Current f : filters) {
            if (f.average > f.threshold) {
                return true; // uh oh
            }
        }
        return false;
    }
}
