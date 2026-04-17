package frc.robot.subsystems.spindexer;

public class SpindexerConstants {
    public static final double spindexerVelocityWithBall = 6.0; // rps (for counting balls)
    public static final double currentLimit = 40; // A
    public static final double spindexerForwardVoltage = 1.00; // Volts (set low for testing)
    public static final double spindexerReverseVoltage = -1.00; // Volts
    public static final double GEAR_RATIO = 27.0; // unused & both motors have same gearing

    public static final double CURRENT_SPIKE_LIMIT = 40;
    public static final double CURRENT_TIME_LIMIT = 1.0; //s
    public static final double JAM_CURRENT_THRESHOLD = 75.0; // A
    public static final double JAM_DEBOUNCE_TIME = 0.3; // seconds
    public static final double REVERSE_DEBOUNCE_TIME = 0.25; // seconds
}
