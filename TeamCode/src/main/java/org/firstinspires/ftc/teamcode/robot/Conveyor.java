package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.mapping.MotorMap;

/***
 * The class responsible for the running of the conveyor mechanism on the robot
 */
public class Conveyor extends Mechanism {
    private CRServo conveyorServo;
    private final double REST_POWER = -0.05;

    /***
     * Conveyor constructor
     * @param map the hardware map for the robot
     */
    public Conveyor(HardwareMap map, Telemetry telemetry, Robot robot){
        super(telemetry, robot);
        this.conveyorServo = map.get(CRServo.class, MotorMap.CONVEYOR_CRSERVO.getMotorName());
    }

    /***
     * Convey rings from the intake to the shooting mechanism, with a @power [0-1]
     * @param power the power with which to push the rings through the conveyor
     */
    public void convey(double power){
        this.conveyorServo.setPower(power);
    }

    /***
     * Conveys rings from the intake to the shooting mechanism with a preset power
     */
    public void convey(){
        this.convey(1);
    }

    /***
     * Stops the conveyor
     */
    public void stop(){
        this.convey(REST_POWER);
    }

}
