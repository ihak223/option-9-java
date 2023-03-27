// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(Constants.DriveTrainConstants.leftDriveId);
  private final PWMSparkMax m_leftDriveTwo = new PWMSparkMax(Constants.DriveTrainConstants.leftDriveTwoId);
  private final MotorControllerGroup m_leftDriveGroup = new MotorControllerGroup(m_leftDrive, m_leftDriveTwo);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(Constants.DriveTrainConstants.rightDriveId);
  private final PWMSparkMax m_rightDriveTwo = new PWMSparkMax(Constants.DriveTrainConstants.rightDriveTwoId);
  private final MotorControllerGroup m_rightDriveGroup = new MotorControllerGroup(m_rightDrive, m_rightDriveTwo);
  public final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDriveGroup, m_rightDriveGroup);
  //this stuff should probably be in an arm subsystem but who has time for that
  public final PWMSparkMax armMover = new PWMSparkMax(4);
  public final PWMSparkMax armOpener = new PWMSparkMax(5);
 // public final PWMSparkMax armBrake = new PWMSparkMax(6); 



  public DriveTrainSubsystem() {
    m_rightDriveGroup.setInverted(false);
    m_leftDriveGroup.setInverted(true);
  }

  @Override
  public void periodic() {
    
  }

}
