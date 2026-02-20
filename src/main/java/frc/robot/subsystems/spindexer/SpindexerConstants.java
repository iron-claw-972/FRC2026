package frc.robot.subsystems.spindexer;

public class SpindexerConstants {
    public static final double gearRatio = 1.0 / 27.0;
    // TODO: measure actual velocity with/without ball to tune threshold
    public static final double spindexerVelocityWithBall = 6.0 * gearRatio; // output rps at full power
    public static final double spindexerMaxPower = 1.0;
    public static final int currentLimit = 40; // amps
}
