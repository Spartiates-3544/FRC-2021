package frc.robot;

import edu.wpi.first.wpilibj.Joystick;


public class Buttons {
    private final Joystick m_stick = new Joystick(0);
    private final Methods m_methods = new Methods();
    private static final int A = 1;
    private static final int B = 2;
    private static final int X = 3;
    private static final int Y = 4;

    //Intake button code
    public void IntakeButtons() {
        if (m_stick.getRawButton(A)) {
            m_methods.IntakeOn(0.25);
        } else {
            m_methods.IntakeOff();
        }
    }

    //Intake Arm button code
    public void IntakeArmButtons() {
        //Initialises the Button int (needed for the switch)
        int Button = 0;

        //Couple if statements for setting the Button int when pressing buttons
        if (m_stick.getRawAxis(5) < -0.1) {
            Button = 1;
        }

        if (m_stick.getRawAxis(5) > 0.1) {
            Button = 2;
        }

        //Switch statement for executing code bases on the Button int
        switch (Button) {
            case 1:
            m_methods.IntakeArmThrottle(m_stick.getRawAxis(5) * 0.50);
                break;
        
            case 2:
            m_methods.IntakeArmThrottle(m_stick.getRawAxis(5) * 0.50);
                break;

            default:
            m_methods.IntakeArmThrottle(0.00);
                break;
        }

    }

    //Conveyor button code
    public void ConveyorButtons() {
        if (m_stick.getRawButton(Y)) {
            m_methods.ConveyorOn(0.25);
        } else {
            m_methods.ConveyorOff();
        }
    }

    //Thrower button code
    public void ThrowerButtons() {
        if (m_stick.getRawButton(B)) {
            m_methods.ThrowerOn(0.75);
        } else {
            m_methods.ThrowerOff();
        }
    }



}