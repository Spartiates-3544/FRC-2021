package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// Methods for operating different parts of the robot (Intake, Thrower, etc.)
public class Methods {
//Change this for speeds     
double m_intakeSpeed = 0.5;
double m_intakeArmSpeed = 0.125;

//Unless needed, don't touch anything from this point on (especially future me lol)
private final WPI_VictorSPX m_intakeRoller = new WPI_VictorSPX(7);
private final WPI_TalonSRX m_intakeArm = new WPI_TalonSRX(11);

//Intake Switch On
public void IntakeOn() {
     m_intakeRoller.set(m_intakeSpeed);
}

//Intake Switch Off
public void IntakeOff() {
     m_intakeRoller.set(0);
}

//Intake arm switch up
public void IntakeArmUp() {
     m_intakeArm.set(m_intakeArmSpeed);
}

//Intake arm switch down
public void IntakeArmDown() {
     m_intakeArm.set(-m_intakeArmSpeed);
}

}