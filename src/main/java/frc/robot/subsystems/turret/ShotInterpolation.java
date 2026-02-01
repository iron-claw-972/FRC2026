package frc.robot.subsystems.turret;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public class ShotInterpolation {
    public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shooterPowerMap = new InterpolatingDoubleTreeMap();

    static{
        timeOfFlightMap.put(0.0, 0.67);
        timeOfFlightMap.put(1.0, 0.67);

        shooterPowerMap.put(0.0, 1.0);
        shooterPowerMap.put(1.0, 1.0);
    }
}
