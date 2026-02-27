package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hood.HoodConstants;

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

        //hoodAngleMap.put(HoodConstants.MAX_ANGLE, HoodConstants.MAX_ANGLE);
        hoodAngleMap.put(81.3, 70.25);
        hoodAngleMap.put(79.0, 65.9);
        hoodAngleMap.put(58.5, 48.5);
        //hoodAngleMap.put(1.0, Units.degreesToRadians(90));

        exitVelocityMap.put(0.0, 0.0);
        exitVelocityMap.put(1.0, 2.2);
        exitVelocityMap.put(2.0, 4.4);
        exitVelocityMap.put(7.0, 12.0);
        exitVelocityMap.put(7.8, 15.2);
        exitVelocityMap.put(7.78, 16.8);
        exitVelocityMap.put(7.9, 17.1);
        exitVelocityMap.put(8.0, 17.9);
        exitVelocityMap.put(8.08, 19.0);
        exitVelocityMap.put(25.0, 25.0* 3.2);
        //exitVelocityMap.put(null, null);
    }
}
