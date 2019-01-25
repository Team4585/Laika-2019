package org.huskyrobotics.frc2018.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
//implement husky subsystem

public class Flipper implements HuskySubsystem {

    private int m_channel = 0;
    private double m_pulseDuration = 1.00; //from 0.01 to 2.55 seconds
    private Solenoid m_sol = new Solenoid(m_channel);
    /**
     * Extends the clamps so they can clamp on.
     */
    public void extendArms() {

    }

    /**
     * Clamps onto the platform so winch can pull robot up.
     */
    public void clamp() {
        //The duration of the pulse, from 0.01 to 2.55 seconds. See http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Solenoid.html#setPulseDuration(double)
        m_sol.setPulseDuration(m_pulseDuration);
        //turns the output on. See http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Solenoid.html#set(boolean)
        m_sol.set(true);
        //starts the pulse based on the duration see setPulseDuration above. See http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Solenoid.html#startPulse()
        m_sol.startPulse();

        //checks if the solenoid output is true.
        if(m_sol.get()) {
            System.out.println("Solenoid is On");
        } else {
            System.out.println("Solenoid is Off");
        }

        /*when to close/stop the solenoid output.
        if(something) {
            sol.close();
        }
        */
    }

//    /**
//     * Pulls the main part of the robot onto the platform.
//     */
//    public void winch() {
//
//
//    }

<<<<<<< HEAD
    }

    @Override
    public void autoInit() {

    }

    @Override
    public void doAuto() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void doTeleop() {

    }
=======
>>>>>>> 997931f0609a1e08c6e2412a34f98c3d46328f22
}

