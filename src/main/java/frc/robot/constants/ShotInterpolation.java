package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotInterpolation {
    public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shooterPowerMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    
    public static final InterpolatingDoubleTreeMap exitVelocityMap = new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap shooterVelocityMap = new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap newHoodMap = new InterpolatingDoubleTreeMap();

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
        exitVelocityMap.put( 21.0, 19.0);

        exitVelocityMap.put(9.90, 14.0);
        exitVelocityMap.put(9.95, 16.5);
        exitVelocityMap.put(10.0, 19.2);
        exitVelocityMap.put(11.0, 26.0);
        exitVelocityMap.put(25.0, 25.0* 3.2);

        shooterVelocityMap.put(1.49, 11.5);
        shooterVelocityMap.put(2.09, 12.5);
        shooterVelocityMap.put(2.95, 13.5);
        shooterVelocityMap.put(5.05, 16.0);
        shooterVelocityMap.put(5.79, 17.0);
        shooterVelocityMap.put(4.07, 15.5);

        shooterVelocityMap.put(0.0, 9.55);
        shooterVelocityMap.put(25.0, 43.44);

        newHoodMap.put(1.49, 72.0);
        newHoodMap.put(2.09, 70.0);
        newHoodMap.put(2.95, 68.0);
        newHoodMap.put(5.05, 60.0);
        newHoodMap.put(5.79, 59.0);
        newHoodMap.put(4.07, 65.0);

        newHoodMap.put(0.0, 75.9);
        newHoodMap.put(27.99, 0.0);

    }
}
