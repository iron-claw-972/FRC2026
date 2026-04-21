package frc.robot.subsystems.PowerControl;

import java.util.LinkedHashMap;
import java.util.Map;

public class BreakerConstants {
    public static final Map<Double, Double> THRESHOLDS = new LinkedHashMap<>();
    static {
        THRESHOLDS.put(1.0, 6.0 * 120); // breaker default at 120
        THRESHOLDS.put(4.0, 3.4 * 120);
        THRESHOLDS.put(10.0, 2.0 * 120);
        THRESHOLDS.put(20.0, 1.6 * 120);
        THRESHOLDS.put(30.0, 1.5 * 120);
    }

    public static final double WARNING_PERCENTAGE = 0.6; // percent that the system reacts to approaching thresholds

    // ports
    public static int[] DRIVETRAIN_PORTS = { 8, 9, 10, 11, 18, 19, 0, 1 }; // bls, bld, fld, fls, frs, frd, brd, brs
    public static int[] TURRET_PORTS = { 2 };
    public static int[] INTAKE_PORTS = { 15, 14, 13 }; // right, left, roller
    public static int[] SHOOTER_PORTS = { 3, 4 }; // left, right
    public static int[] HOOD_PORTS = { 5 }; // shooter
    public static int[] SPINDEXER_PORTS = { 12 }; // spindexer (unupdated on sheets)
}
