// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveTrainAutonomous extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final Timer m_timer;
  //private final DigitalInput digitalInput;
  private ADIS16470_IMU gyro = RobotContainer.gyro;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public DriveTrainAutonomous(DriveTrainSubsystem driveTrainSubsystem, Timer m_timer, ADIS16470_IMU gyro) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    //this.digitalInput = dInput;
    this.m_timer = m_timer;
    this.gyro = gyro;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled. 
  @Override
  public void initialize() {

    System.out.println("The autonomous drivetrain command thing is working! This is good.");
    //starts timer
    m_timer.reset();
    m_timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (m_timer.get() < 0.5){
      driveTrainSubsystem.m_robotDrive.arcadeDrive(0.7, 0.0); 
    } 
    //reverse and go away from the grid
    while (m_timer.get() > 3.5 && m_timer.get() < 4.65){
      driveTrainSubsystem.m_robotDrive.arcadeDrive(-0.8, 0.0);
    }
    //turn right towards drivers station
    while (m_timer.get() > 4.65 && m_timer.get() < 5.0){
      driveTrainSubsystem.m_robotDrive.arcadeDrive(0.5, 0.2);
    }
    //drive straight parallel to the charge station
    while (m_timer.get() > 5.0 && m_timer.get() < 5.5){
      driveTrainSubsystem.m_robotDrive.arcadeDrive(0.7, 0.0);
    }
    //turn left toward charge station
    while (m_timer.get() > 5.5 && m_timer.get() < 5.75){
      driveTrainSubsystem.m_robotDrive.arcadeDrive(0.5, -0.2);
    }
    //drive onto charge station
    while (m_timer.get() > 5.75 && m_timer.get() < 6.3){
      driveTrainSubsystem.m_robotDrive.arcadeDrive(0.7, 0.0);
    }

    //theoritically this should cause it move back and forth to balance on the charge station, ultimately stopping when autonomous period ends
    if (gyro.getAngle() > 0.0){
      driveTrainSubsystem.m_robotDrive.arcadeDrive(0.2,0.0);
    }
    if (gyro.getAngle() < 0.0){
      driveTrainSubsystem.m_robotDrive.arcadeDrive(-0.2,0.0);
    }
    
    
  
    

    //System.out.println("get : " + digitalInput.get());

   // if (digitalInput.get() == true){
  //    driveTrainSubsystem.m_robotDrive.arcadeDrive(0.7, 0.0);
  //  }
   // else {
  //    driveTrainSubsystem.m_robotDrive.arcadeDrive(0.7, 0.0);
  //  }

    
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
