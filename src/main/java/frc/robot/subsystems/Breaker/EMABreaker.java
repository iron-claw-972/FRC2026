package frc.robot.subsystems.Breaker;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class EMABreaker extends SubsystemBase {

    private static class Current {
        double tau;
        double alpha; // how much of the error we correct per loop
        double average = 0;
        double threshold;
    }

    PowerDistribution pDis = new PowerDistribution();

    double[] subsystemCurrents;

    private List<Current> filters = new ArrayList<>(); // contains currents with their alphas and thresholds
    private List<Current> subsystems = new ArrayList<>();

    public EMABreaker() {
        for (Map.Entry<Double, Double> entry : BreakerConstants.THRESHOLDS.entrySet()) {
            double tau = entry.getKey(); // sec
            double threshold = entry.getValue(); // A

            Current w = new Current(); // create a filter for the threshold
            w.tau = tau;
            w.threshold = threshold;
            w.alpha = 1 - Math.exp(-Constants.LOOP_TIME / tau); // 1 - e^(-0.02/1) = 0.0198, 1 - e^(-0.02/2) = 0.00995 

            filters.add(w);
        }

        // subsystems
        for (int i=0; i<pDis.getNumChannels(); i++) {
            double tau = 1.0;
            double threshold = i;
            Current w = new Current();
            w.tau = tau;
            w.threshold = threshold;
            w.alpha = 1 - Math.exp(-Constants.LOOP_TIME / tau); // 1 - e^(-0.02/1) = 0.

            subsystems.add(w);
        }
    }

    @Override
    public void periodic() {
        double current = getCurrentFromPowerDistribution();
        // this is total current averages
        for (Current f : filters) {
            // new avg = old avg + fractionAlpha * difference
            f.average += f.alpha * (current - f.average);
            Logger.recordOutput("Breaker/IntervalAverage/" + f.tau, f.average);
        }

        // this is getting currents coming out of all the ports from PDH (big thing under robot all the wires come out of)
        subsystemCurrents = getAllCurrentFromPowerDistribution();
        
        // this should average out all ports
        for (Current s : subsystems) {
            s.average += s.alpha * (subsystemCurrents[(int) s.threshold] - s.average);
        }

        // this should use updated port averages and sum them to get drivetrain average draw for 1 tau (can add more later)
        Logger.recordOutput("Breaker/DrivetrainAverageDraw", getAverageCurrentDraw(BreakerConstants.DRIVETRAIN_PORTS));
        Logger.recordOutput("Breaker/SpindexerDraw", getAverageCurrentDraw(BreakerConstants.SPINDEXER_PORTS));
        Logger.recordOutput("Breaker/ShooterDraw", getAverageCurrentDraw(BreakerConstants.SHOOTER_PORTS));
        Logger.recordOutput("Breaker/IntakeDraw", getAverageCurrentDraw(BreakerConstants.INTAKE_PORTS));
        Logger.recordOutput("Breaker/TurretDraw", getAverageCurrentDraw(BreakerConstants.TURRET_PORTS));
        Logger.recordOutput("Breaker/HoodDraw", getAverageCurrentDraw(BreakerConstants.HOOD_PORTS));

        // total stuff
        Logger.recordOutput("Breaker/TotalCurrent", current);
        Logger.recordOutput("Breaker/OverCurrent", isOverCurrent());
    }

    public double getAverageCurrentDraw(int[] ports) {
        double sum = 0;
        for (int number : ports) {
            sum += subsystems.get(number - 1).average; // the list starts at zero, so ports will be shifted
        }
        return sum;
    }

    public double getCurrentFromPowerDistribution() {
        return pDis.getTotalCurrent(); // not using .getCurrent() and then an arguement for the port you can get just one port
    }

    public double[] getAllCurrentFromPowerDistribution() {
        return pDis.getAllCurrents();
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
