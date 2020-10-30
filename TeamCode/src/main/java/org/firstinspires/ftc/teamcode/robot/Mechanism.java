package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/***
 *
 */
public abstract class Mechanism {
    protected Telemetry telemetry;

    public Mechanism(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    

}
