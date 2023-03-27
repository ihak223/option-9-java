package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import java.lang.Math;

public class DriveMath {

    public static double calculateSpeed(Joystick js)
    {
        // 
        return Math.pow(js.getY(), 2) * ( (js.getRawAxis(3) * -0.25) + 0.75);
    }
}
