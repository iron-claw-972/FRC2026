package frc.robot.subsystems.turret;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotInterpolation {
    public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    public static Object timeOfFlight;

    static{
        timeOfFlightMap.put(0.0, 0.67);
        timeOfFlightMap.put(1.0, 0.67);
    }
}
