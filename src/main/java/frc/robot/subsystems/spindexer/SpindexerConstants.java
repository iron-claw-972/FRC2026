package frc.robot.subsystems.spindexer;

public class SpindexerConstants {
    public static final double spindexerVelocityWithBall = 6.0; // rps (for counting balls)
    public static final double currentLimit = 40; // Amps
    public static final double spindexerMaxPower = 0.75; 
    public static final double spindexerReversePower = -1.00;
    public static final double CURRENT_SPIKE_LIMIT = 150.0;
    public static final double CURRENT_TIME_LIMIT = 1.0; //seconds
    public static final double JAM_CURRENT_THRESHOLD = 75.0; // Amps
    public static final double JAM_DEBOUNCE_TIME = 0.3; // seconds, how long higher than threshold until debounce
    public static final double REVERSE_TIME = 0.25; // seconds, how long to reverse for
}
