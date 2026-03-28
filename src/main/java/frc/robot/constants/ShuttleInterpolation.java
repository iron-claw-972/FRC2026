package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShuttleInterpolation {
    public static final InterpolatingDoubleTreeMap newHoodMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shooterVelocityMap = new InterpolatingDoubleTreeMap();
    /*
     * guide to tuning:
     * modify the left values in the parenthesis to change the distance of the shot.
     * setup the robot at that distance and shoot. Then modifty the right value to
     * be more or less until it hits target perfectly.
     * Repeat
     * OR
     * Run robot, estimate distance, and tweak values when nobodies looking until it
     * works.
     */
    static {
        // we can be less aggressive: y = 0.65 * (1.34959x + 9.79618)
        // will likely be this that requires tuning.
        shooterVelocityMap.put(0.0, 6.2075);
        shooterVelocityMap.put(1.00, 7.475);
        shooterVelocityMap.put(1.49, 7.475);
        shooterVelocityMap.put(2.09, 8.125);
        shooterVelocityMap.put(2.95, 8.775);
        shooterVelocityMap.put(4.07, 10.075);
        shooterVelocityMap.put(5.05, 10.855);
        shooterVelocityMap.put(5.79, 11.7);
        shooterVelocityMap.put(25.0, 28.236);

        // always shoot at low angle to ground.
        newHoodMap.put(0.0, 80.0);
        newHoodMap.put(1.00, 80.0);
        newHoodMap.put(1.49, 80.0);
        newHoodMap.put(2.09, 80.0);
        newHoodMap.put(2.95, 80.0);
        newHoodMap.put(5.05, 80.0);
        newHoodMap.put(5.79, 80.0);
        newHoodMap.put(4.07, 80.0);
        newHoodMap.put(27.99, 80.0);
    }

}
