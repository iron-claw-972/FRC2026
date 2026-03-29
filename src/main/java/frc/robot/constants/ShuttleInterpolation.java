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
        shooterVelocityMap.put(0.0, 9.0);
        shooterVelocityMap.put(4.0, 12.8);
        shooterVelocityMap.put(7.6, 19.0);


        // always shoot at low angle to ground.
        newHoodMap.put(0.0, 60.0);
        newHoodMap.put(27.99, 60.0);
    }

}
