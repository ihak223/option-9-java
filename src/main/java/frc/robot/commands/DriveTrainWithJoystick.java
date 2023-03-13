// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveTrainWithJoystick extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrainSubsystem driveTrainSubsystem;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  ShuffleboardTab driver;
  //ADIS16470_IMU gyro;
  //NetworkTableEntry kPEntry;

  public DriveTrainWithJoystick(DriveTrainSubsystem driveTrainSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    //gyro
    //this.gyro = gyro2;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("The joystick drivetrain command thing is working! This is good.");
    driver =Shuffleboard.getTab("robot_things");
    UsbCamera camera = CameraServer.startAutomaticCapture();
    driver.add(camera).withSize(6,6);
    
    //Gyro in shuffleboard
    //driver.add(gyro).withSize(2, 2);
    //driver.addNumber("Gyro Angle", gyro::getAngle).withSize(3,3);
  }

  
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //for broken joystick, gety, getx * -1
    //go when the joystick goes. all the math is for the sensitivity toggle (the slider thing on joystick) there's probably a more efficient way to do the same math but i do not care.
    driveTrainSubsystem.m_robotDrive.arcadeDrive(RobotContainer.joystick.getY() * ((((RobotContainer.joystick.getRawAxis(3) * -1) + 1) * 0.25) + .5), RobotContainer.joystick.getX() * -0.6);
    //sensitivity diagnostics lol
    //for reference of future coders, this is a very nice way to display a number without looking at it on the riolog awkwardly.
    SmartDashboard.putNumber("Joystick Y Axis Input", RobotContainer.joystick.getY());
    SmartDashboard.putNumber("Joystick X Axis Input", RobotContainer.joystick.getX());
    SmartDashboard.putNumber("Joystick Sensitivity Input", RobotContainer.joystick.getRawAxis(3));
    SmartDashboard.putNumber("Current Speed Output", RobotContainer.joystick.getY() * ((((RobotContainer.joystick.getRawAxis(3) * -1) + 1) * 0.25) + .5));
    
    //gyro code for driver orientation 
   /* double error = gyro.getAngle();
    double kP = kPEntry.getDouble(0);
    double speed = (kP*error);
    if (speed > 0.7){
      speed = 0.7;
    }
    if (speed < -0.7){
      speed = -0.7;
    }
    
    driveTrainSubsystem.m_robotDrive.tankDrive(-speed, speed); */



    //raises arm at a really slow speed (o.1)
    if (RobotContainer.joystick.getRawButton(1)){
      driveTrainSubsystem.armMover.set(-1.0);
    }
    //lowers arm at a really slow speed
    if (RobotContainer.joystick.getRawButton(2)){
      driveTrainSubsystem.armMover.set(1.0);
    }

    //opens arm claw at a really slow speed (o.2)
    if (RobotContainer.joystick.getRawButton(6)){
      driveTrainSubsystem.armOpener.set(0.5);
    }
    //closes arm claw at a really slow speed 
    if (RobotContainer.joystick.getRawButton(4)){
      driveTrainSubsystem.armOpener.set(-0.65);
    }

    //if nots
    if ((!RobotContainer.joystick.getRawButton(1)) && (!RobotContainer.joystick.getRawButton(2))) {
      driveTrainSubsystem.armMover.stopMotor();
    }
    if ((!RobotContainer.joystick.getRawButton(4)) && (!RobotContainer.joystick.getRawButton(6))) {
      driveTrainSubsystem.armOpener.stopMotor();
    }

    //emergency stops
    if (RobotContainer.joystick.getRawButton(12)){
      driveTrainSubsystem.armMover.stopMotor();
    }
    if (RobotContainer.joystick.getRawButton(11)){
      driveTrainSubsystem.armOpener.stopMotor();
    }
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
