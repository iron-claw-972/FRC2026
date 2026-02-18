package frc.robot.subsystems.turret;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShotInterpolation {
    public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shooterPowerMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    
    public static final InterpolatingDoubleTreeMap exitVelocityMap = new InterpolatingDoubleTreeMap();

    static{
        timeOfFlightMap.put(0.0, 0.67);
        timeOfFlightMap.put(1.0, 0.67);

        shooterPowerMap.put(0.0, 1.0);
        shooterPowerMap.put(1.0, 1.0);

        hoodAngleMap.put(0.0, Units.degreesToRadians(90));
        hoodAngleMap.put(1.0, Units.degreesToRadians(90));

        //TODO: find actual values from video motion
        exitVelocityMap.put(1.0, 2.0);
        exitVelocityMap.put(2.0, 4.0);
    }
}
