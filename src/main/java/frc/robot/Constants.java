// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveTrainConstants{
        public static final int leftDriveId = 0;
        public static final int leftDriveTwoId = 1;
        public static final int rightDriveId = 2;
        public static final int rightDriveTwoId = 3;
    }
    public static final class JoystickConstants{
        public static final int sliderAxis = 3;      // Raw Axis to change sensitivity
        public static final int raiseArmButton = 1;  // Raw Button to move arm up
        public static final int lowerArmButton = 2;  // Raw Button to move arm down
        public static final int openClawButton = 6;  // Raw Button to open claw
        public static final int closeClawButton = 4; // Raw Button to clone claw
    }
}
