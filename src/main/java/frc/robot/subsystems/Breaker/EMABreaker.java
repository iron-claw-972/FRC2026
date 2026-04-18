package frc.robot.subsystems.Breaker;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.PowerDistributionJNI;
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

    double[] subsystemCurrents;

    private List<Current> filters = new ArrayList<>(); // contains currents with their alphas and thresholds
    private List<Current> subsystems = new ArrayList<>();

    private int count = 0;
    public EMABreaker() {
        for (Map.Entry<Double, Double> entry : BreakerConstants.THRESHOLDS.entrySet()) {
            double tau = entry.getKey(); // sec
            double threshold = entry.getValue(); // A

            Current w = new Current(); // create a filter for the threshold
            w.threshold = threshold;
            w.alpha = 1 - Math.exp(-Constants.LOOP_TIME / tau); // 1 - e^(-0.02/1) = 0.0198, 1 - e^(-0.02/2) = 0.00995 

            filters.add(w);
        }

        // subsystems
        for (int i=0; i<pDis.getNumChannels(); i++) {
            double tau = 1.0;
            double threshold = i;
            Current w = new Current();
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
            Logger.recordOutput("Breaker/Filter Port" + f.threshold + "Avg", f.average);
        }

        // this is getting currents coming out of all the ports from PDH (big thing under robot all the wires come out of)
        subsystemCurrents = getAllCurrentFromPowerDistribution();
        for (Current s : subsystems) {
            s.average += s.alpha * (subsystemCurrents[(int) s.threshold] - s.average);
            Logger.recordOutput("Breaker/Current Port" + s.threshold + "Avg", s.average);
        }

        Logger.recordOutput("Breaker/TotalCurrent", current);
        Logger.recordOutput("Breaker/OverCurrent", isOverCurrent());

        // subsystems
        Logger.recordOutput("Breaker/DrivetrainDraw", getDrivetrainCurrentDraw(subsystemCurrents));
        Logger.recordOutput("Breaker/SpindexerDraw", getSpindexerCurrentDraw(subsystemCurrents));
        Logger.recordOutput("Breaker/ShooterDraw", getShooterCurrentDraw(subsystemCurrents));
        Logger.recordOutput("Breaker/IntakeDraw", getIntakeCurrentDraw(subsystemCurrents));
        Logger.recordOutput("Breaker/TurretDraw", getTurretCurrentDraw(subsystemCurrents));
        Logger.recordOutput("Breaker/HoodDraw", getHoodCurrentDraw(subsystemCurrents));
    }

    public double getDrivetrainCurrentDraw(double[] subsystemCurrents) {
        double sum = 0;
        for (int number : BreakerConstants.DRIVETRAIN_PORTS) {
            sum += subsystemCurrents[number];
        }
        return sum;
    }

    public double getSpindexerCurrentDraw(double[] subsystemCurrents) {
        double sum = 0;
        for (int number : BreakerConstants.SPINDEXER_PORTS) {
            sum += subsystemCurrents[number];
        }
        return sum;
    }

    public double getShooterCurrentDraw(double[] subsystemCurrents) {
        double sum = 0;
        for (int number : BreakerConstants.SHOOTER_PORTS) {
            sum += subsystemCurrents[number];
        }
        return sum;
    }

    public double getIntakeCurrentDraw(double[] subsystemCurrents) {
        double sum = 0;
        for (int number : BreakerConstants.INTAKE_PORTS) {
            sum += subsystemCurrents[number];
        }
        return sum;
    }

    public double getTurretCurrentDraw(double[] subsystemCurrents) {
        double sum = 0;
        for (int number : BreakerConstants.TURRET_PORTS) {
            sum += subsystemCurrents[number];
        }
        return sum;
    }

    public double getHoodCurrentDraw(double[] subsystemCurrents) {
        double sum = 0;
        for (int number : BreakerConstants.HOOD_PORTS) {
            sum += subsystemCurrents[number];
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
