package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import java.lang.Math;

public class DriveMath {

    public static double calculateSpeed(Joystick js)
    {
        // 
        if (js.getY() >= 0){
        return Math.pow(js.getY(), 2) * ( (js.getRawAxis(3) * -0.25) + 0.75);
        } else return Math.pow(js.getY(), 2) * -1 * ( (js.getRawAxis(3) * -0.25) + 0.75);
    }
    public static double calculateTurnSpeed(Joystick js)
    {
        // 
        if (js.getX() >= 0){
        return Math.pow(js.getX(), 2) * -0.6;
        } else return Math.pow(js.getX(), 2) * 0.6;
    }
}
