package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.mapping.MechanismMap;
import org.firstinspires.ftc.teamcode.robot.mapping.MotorMap;

/***
 * The class responsible for running and controlling the shooting mechanisms on the robot
 */
public class Shooter extends Mechanism {
    private DcMotor whackyStick;

    /***
     * Shooter constructor
     * @param map hardware map of the robot
     */
    public Shooter(HardwareMap map, Telemetry telemetry, Robot robot){
        super(telemetry, robot);
        this.whackyStick = map.get(DcMotor.class, MotorMap.SHOOTER_DC.getMotorName());
        this.whackyStick.setDirection(DcMotorSimple.Direction.REVERSE);
//        this.getInterlock().registerInterlock(this.robot.getMechanism(MechanismMap.SHOOTER.getName()));
    }

    /***
     * Shoots a ring with a power
     * @param power the power with which to shoot a ring [0-1]
     */
    public void shoot(double power){
        if (!this.whackyStick.isBusy()){
            this.whackyStick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.whackyStick.setTargetPosition(9 * 1440 / 9);
            this.whackyStick.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.whackyStick.setPower(power);
        }
    }

    /***
     * Shoots a ring with a preset power
     */
    public void shoot() {
        shoot(1);
    }

    /***
     * Loading the ring into the shooting mechanism
     */
    public void feedMe(){

    }
}
