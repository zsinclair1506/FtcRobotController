package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.lib.Interlock;
import org.firstinspires.ftc.teamcode.robot.mapping.InterlockMap;

import java.util.HashMap;


/***
 *
 */
public abstract class Mechanism {
    protected Telemetry telemetry;
    protected Robot robot;
    protected HashMap<String, Interlock> interlocks = new HashMap<>();

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
     * Gets the interlock for this mechanism.
     * @return the interlock
     */
    public Interlock getInterlock(InterlockMap.Actions key){
        return this.interlocks.get(key);
    }

    public abstract void updateInterlocks();
}
