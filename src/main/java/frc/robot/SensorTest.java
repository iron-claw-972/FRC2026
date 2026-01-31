package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SensorTest extends TimedRobot {

    private final CANcoder m_encoder = new CANcoder(0);

    @Override
    public void robotPeriodic() {
        //position in rotations
        double position = m_encoder.getAbsolutePosition().getValueAsDouble();
        
        //velocity in rotations(per sec)
        double velocity = m_encoder.getVelocity().getValueAsDouble();

        SmartDashboard.putNumber("encoder position rot", position);
        SmartDashboard.putNumber("encoder velocity rps", velocity);
    }
}
