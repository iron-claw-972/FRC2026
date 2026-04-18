package frc.robot.subsystems.Breaker;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

public class BreakerConstants {
    public static final Map<Double, Double> THRESHOLDS = new LinkedHashMap<>();
    static {
        THRESHOLDS.put(1.0, 600.0);
        THRESHOLDS.put(2.0, 470.0);
        THRESHOLDS.put(3.0, 380.0);
        THRESHOLDS.put(4.0, 340.0);
        THRESHOLDS.put(5.0, 280.0);
        THRESHOLDS.put(7.0, 240.0);
        THRESHOLDS.put(10.0, 200.0);
        THRESHOLDS.put(15.0, 175.0);
        THRESHOLDS.put(20.0, 160.0);
        THRESHOLDS.put(30.0, 150.0);
        THRESHOLDS.put(100.0, 130.0);
        THRESHOLDS.put(180.0, 120.0);
        THRESHOLDS.put(200.0, 110.0);
        THRESHOLDS.put(500.0, 105.0);
    }
}
