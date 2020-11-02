package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/***
 *
 */
public abstract class Mechanism {
    protected Telemetry telemetry;
    protected Robot robot;

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

}
