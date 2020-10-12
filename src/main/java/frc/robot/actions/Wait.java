package frc.robot.actions;

import frc.robot.subsystems.*;

public class Wait extends SingleAction {
    private long startTime;
    private long waitTime;

    public Wait(Subsystem subsystem, double waitTime) { //Wait time in seconds
        super(subsystem);
        this.waitTime = (long) (waitTime * 1000);
    }

    public void init() {
        startTime = System.currentTimeMillis();
    }

    public void loop() {
        if (System.currentTimeMillis() - startTime >= waitTime) {
            setDone();
        }
    }
}