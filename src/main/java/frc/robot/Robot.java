// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.examples.ramsetecommand.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Robot extends TimedRobot {

  //Speed Controller Groups (For motors/DifferentialDrive)
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(new WPI_TalonFX(3),
                               new WPI_TalonFX(4));

  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(new WPI_TalonFX(1),
                               new WPI_TalonFX(2));

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  private final Joystick m_stick = new Joystick(0);
  private final Buttons m_buttons = new Buttons();
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }



  //Put any autonomous initialisation code here
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    
    //TODO IF NEEDED ONLY
    //m_robotDrive.setSafetyEnabled(false);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  
    }


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }


  @Override
  public void teleopPeriodic() {

    // Drive with arcade drive (Modified for reversed rocket league style controls)
    double leftTrigger = m_stick.getRawAxis(2);
    double rightTrigger = m_stick.getRawAxis(3);
    double rotation = rightTrigger - leftTrigger;

    m_robotDrive.arcadeDrive(-m_stick.getY() * 0.75, rotation * 0.75);
    m_robotDrive.feed();
    m_buttons.IntakeArmButtons();
    m_buttons.IntakeButtons();
    m_buttons.ConveyorButtons();
    m_buttons.ThrowerButtons();
  }

  @Override
  public void autonomousPeriodic() {

  }
 

}
