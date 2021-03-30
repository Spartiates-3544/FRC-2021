package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//cral
// Methods for operating different parts of the robot (Intake, Thrower, etc.)
public class Methods {
//Unless needed, don't touch anything from this point on (especially future me lol)
private final WPI_VictorSPX m_intakeRoller = new WPI_VictorSPX(7);
private final WPI_TalonSRX m_intakeArm = new WPI_TalonSRX(11);
private final WPI_VictorSPX m_conveyorLow = new WPI_VictorSPX(8);
private final WPI_VictorSPX m_conveyorHigh = new WPI_VictorSPX(9);
private final WPI_TalonSRX m_shooter1 = new WPI_TalonSRX(12);
private final WPI_TalonSRX m_shooter2 = new WPI_TalonSRX(13);

//PID controller for shooter
// private static final double kP = -.075;
// private static final double kI = -0.00;
// private static final double kD = -0.0;
// PIDController shooter_pid = new PIDController(kP, kI, kD); 

//Intake Switch On
public void IntakeOn(double m_intakeSpeed) {
     m_intakeRoller.set(m_intakeSpeed);
}

//Intake Switch Off
public void IntakeOff() {
     m_intakeRoller.set(0);
}

//Intake arm throttle
public void IntakeArmThrottle(double m_intakeArmThrottle) {
     m_intakeArm.set(m_intakeArmThrottle);
}

//Conveyor switch on
public void ConveyorOn(double m_conveyorSpeed) {
     m_conveyorHigh.set(-m_conveyorSpeed);
     m_conveyorLow.set(m_conveyorSpeed);
}

//Conveyor switch off
public void ConveyorOff() {
     m_conveyorHigh.set(0);
     m_conveyorLow.set(0);
}

//Thrower mechanism switch on
public void ThrowerOn(double m_throwerSpeed) {
     m_shooter1.set(m_throwerSpeed);
     m_shooter2.set(m_throwerSpeed);
}

//Thrower mechanism switch off
public void ThrowerOff() {
     m_shooter1.set(0);
     m_shooter2.set(0);
}

}