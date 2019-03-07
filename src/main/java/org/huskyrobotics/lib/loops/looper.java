package org.huskyrobotics.lib.loops;
//254 Loop system

import org.huskyrobotics.frc2019.Constants;
import org.huskyrobotics.lib.util.RunCrashTracker;


import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;

import java.util.ArrayList;
import java.util.List;

public class looper implements ILooper {
    public final double c_Period = 0.01;

    private boolean m_running;

    private final Notifier m_notifier;
    private final List<Loop> m_loops;
    private final Object m_taskRunningLock = new Object();
    private double m_timestamp = 0;
    private double m_dt = 0;

    private final RunCrashTracker m_runnable = new RunCrashTracker(){

        @Override
        public void runCrashTracked() {
            synchronized(m_taskRunningLock) {
                if(m_running) {
                    double now = Timer.getFPGATimestamp(); //relays the match time

                    for (Loop loop : m_loops) {
                        loop.onLoop(now);
                    }
                }
            }
        }
    };
    public looper() {
        m_notifier = new Notifier(m_runnable);
        m_running = false;
        m_loops = new ArrayList<>();
    }

    @Override
    public synchronized void register(Loop loop) {
        synchronized (m_taskRunningLock) {
            m_loops.add(loop);
        }
    }

    public synchronized void start() {
        if (!m_running) {
            System.out.println("Starting loops");
            synchronized (m_taskRunningLock) {
                m_timestamp = Timer.getFPGATimestamp();
                for (Loop loop : m_loops) {
                    loop.onStart(m_timestamp);
                }
                m_running = true;
            }
            m_notifier.startPeriodic(c_Period);
        }
    }

    public synchronized void stop() {
        if (m_running) {
            System.out.println("Stopping loops");
            m_notifier.stop();
            synchronized (m_taskRunningLock) {
                m_running = false;
                m_timestamp = Timer.getFPGATimestamp();
                for (Loop loop : m_loops) {
                    System.out.println("Stopping " + loop);
                    loop.onStop(m_timestamp);
                }
            }
        }
}

public void outputToSmartDashboard() {
    SmartDashboard.putNumber("looper_dt", m_dt);
    }
}