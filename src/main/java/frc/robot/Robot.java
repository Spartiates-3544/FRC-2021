// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.trajectory.*;
import java.nio.file.*;
import edu.wpi.first.wpilibj.*;
import java.io.*;
//TODO REMOVE .* LIBRARIES & FIND THE ACTUAL GOOD ONES


public class Robot extends TimedRobot {
  private final WPI_TalonFX m_leftMotor1 = new WPI_TalonFX(3);
  private final WPI_TalonFX m_rightMotor1 = new WPI_TalonFX(1);
  private final WPI_TalonFX m_leftMotor2 = new WPI_TalonFX(4);
  private final WPI_TalonFX m_rightMotor2 = new WPI_TalonFX(2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);
  private final Joystick m_stick = new Joystick(0);
  private final Buttons m_buttons = new Buttons();

  // Setting the secondary motors to follow mode
  @Override
  public void robotInit() {

  }
  
  @Override
  public void teleopPeriodic() {
    m_leftMotor2.follow(m_leftMotor1);
    m_rightMotor2.follow(m_rightMotor1); 
    // Drive with arcade drive (Modified for reversed rocket league style controls)
    double leftTrigger = m_stick.getRawAxis(2);
    double rightTrigger = m_stick.getRawAxis(3);
    double rotation = rightTrigger - leftTrigger;

    m_robotDrive.arcadeDrive(-m_stick.getX() * 0.75, rotation * 0.75);
    m_buttons.IntakeArmButtons();
    m_buttons.IntakeButtons();
    m_buttons.ConveyorButtons();
    m_buttons.ThrowerButtons();
  }


  //Put any autonomous initialisation code here
  @Override
  public void autonomousInit() {
    //This is code for importing Pathweaver JSONS into robot code (Change the trajectoryJSON string with the actual path to the JSON). 
    //Path is from the perspective of the roboRIO
    String trajectoryJSON = "paths/YourPath.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }
 
 
 //Code to run every loop in autonomous
  @Override
  public void autonomousPeriodic() {
    
  }


}
