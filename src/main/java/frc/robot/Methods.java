package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// Methods for operating different parts of the robot (Intake, Thrower, etc.)
public class Methods {
//Change this for speeds     
double m_intakeSpeed = 0.5;
double m_intakeArmSpeed = 0.125;
double m_conveyorSpeed = 0.5;

//Unless needed, don't touch anything from this point on (especially future me lol)
private final WPI_VictorSPX m_intakeRoller = new WPI_VictorSPX(7);
private final WPI_TalonSRX m_intakeArm = new WPI_TalonSRX(11);
private final WPI_VictorSPX m_conveyorLow = new WPI_VictorSPX(8);
private final WPI_VictorSPX m_conveyorHigh = new WPI_VictorSPX(9);
private final WPI_TalonSRX m_shooter1 = new WPI_TalonSRX(12);
private final WPI_TalonSRX m_shooter2 = new WPI_TalonSRX(13);



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


//Conveyor switch on
public void ConveyorOn() {
     m_conveyorHigh.set(m_conveyorSpeed);
     m_conveyorLow.set(m_conveyorSpeed);
}

//Conveyor switch off
public void ConveyorOff() {
     m_conveyorHigh.set(0);
     m_conveyorLow.set(0);
}

//Thrower mechanism switch on
public void ThrowerOn() {
     m_shooter1.set(0.5);
     m_shooter2.set(0.5);
}

//Thrower mechanism switch off
public void ThrowerOff() {
     m_shooter1.set(0);
     m_shooter2.set(0);
}

}