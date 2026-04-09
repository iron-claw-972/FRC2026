package frc.robot.subsystems.spindexer;

public class SpindexerConstants {
    public static final double spindexerVelocityWithBall = 6.0; // rps (for counting balls)
    public static final double currentLimit = 40; // A
    public static final double spindexerMaxPower = 0.75; 
    public static final double spindexerReversePower = -1.00;
    public static final double CURRENT_SPIKE_LIMIT = 100;
    public static final double CURRENT_TIME_LIMIT = 1.0; //s
    public static final double JAM_CURRENT_THRESHOLD = 75.0; // A
    public static final double JAM_DEBOUNCE_TIME = 0.3; // seconds
    public static final double REVERSE_DEBOUNCE_TIME = 0.25; // seconds
    public static final double NO_BALLS_DEBOUNCE_TIME = 0.1; // sec
    public static final double BALLS_THRESHOLD_CURRENT_THRESHOLD = 20; // A

    // No balls current is 15-18 A
    // Ranges heavily from 100 A to 15 A. But sitting is around 35 A?
}
