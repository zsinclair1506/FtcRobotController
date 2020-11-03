package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.lib.Interlock;

/***
 *
 */
public abstract class Mechanism {
    protected Telemetry telemetry;
    protected Robot robot;
    protected Interlock interlock = new Interlock();

    /***
     * The constructor for any mechanism. This stores the robot and telemetry for debug and
     * backwards instruction calls.
     * @param telemetry the telemetry to log data to
     * @param robot the robot that this mechanism is attached to
     */
    public Mechanism(Telemetry telemetry, Robot robot){
        this.telemetry = telemetry;
        this.robot = robot;
    }

    /***
     * Gets the interlock object for this mechanism.
     * @return the interlock
     */
    public Interlock getInterlock(){
        return this.interlock;
    }
}
