package frc.robot.subsystems.Breaker;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Breaker extends SubsystemBase {
    PowerDistribution pDis = new PowerDistribution();
    List<Double> currents = new ArrayList<>();

    public Breaker() {
        currents.clear();
    }
    
    @Override
    public void periodic() {
        currents.add(getCurrentFromPowerDistribution());
    }

    public double average(double secondsBackward) {
        // there is a log for every 20ms and thus 50 indexes for each second
        int totalIndexes = (int) secondsBackward * 50;
        List<Double> trimmedCurrents = currents.subList(currents.size()-totalIndexes, currents.size());
        double sum = 0;
        for (double index : trimmedCurrents) {
            sum += index;
        }
        return sum/trimmedCurrents.size();
    }

    public boolean thresholdPassed(double threshold, double secondsBackward) {
        return average(secondsBackward) > threshold;
    }

    public boolean checkThresholdsBroken() {
        if (
            thresholdPassed(BreakerConstants.HALF_SECOND_THRESHOLD_AMPS, 0.5)
            || thresholdPassed(BreakerConstants.ONE_SECOND_THRESHOLD_AMPS, 1)
            || thresholdPassed(BreakerConstants.TWO_SECOND_THRESHOLD_AMPS, 2)
            || thresholdPassed(BreakerConstants.FOUR_SECOND_THRESHOLD_AMPS, 4)
            || thresholdPassed(BreakerConstants.EIGHT_SECOND_THRESHOLD_AMPS, 8)
        ) {
            return true;
        } else {
            return false;
        }
    }

    public double getCurrentFromPowerDistribution() {
        return pDis.getTotalCurrent(); // not using .getCurrent() and then an arguement for the port you can get just one port
    }
}
