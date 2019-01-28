package org.huskyrobotics.lib.util;
//254 Crash Tracking that should be a given everywhere tbh
public abstract class RunCrashTracker implements Runnable {


    @Override
    public final void run() {
        try {
            runCrashTracked();
        } catch (Throwable e) {
            CrashTracker.logThrowableCrash(e);
            throw e;
        }
    }

    public abstract void runCrashTracked();
}