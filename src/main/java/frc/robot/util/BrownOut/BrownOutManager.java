package frc.robot.util.BrownOut;

public class BrownOutManager {
    public BrownOutLevel[] levels = {
        BrownOutConstants.BROWNOUT_LVL_ONE, 
        BrownOutConstants.BROWNOUT_LVL_TWO, 
        BrownOutConstants.BROWNOUT_LVL_THREE, 
        BrownOutConstants.BROWNOUT_LVL_FOUR,
        BrownOutConstants.BROWNOUT_LVL_FIVE,
    };

    public void monitor() {

    }

    public void apply() {}
}
