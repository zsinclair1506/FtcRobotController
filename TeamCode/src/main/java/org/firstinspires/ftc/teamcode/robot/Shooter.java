package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.lib.DCToPositionThread;

/***
 * The class responsible for running and controlling the shooting mechanisms on the robot
 */
public class Shooter extends Mechanism {
    private DcMotor hammerArmMotor;
    private DCToPositionThread shootingThread;

    /***
     * Shooter constructor
     * @param map hardware map of the robot
     */
    public Shooter(HardwareMap map, Telemetry telemetry){
        super(telemetry);
        this.hammerArmMotor = map.get(DcMotor.class, MotorMap.SHOOTER_DC.getMotorName());
        this.shootingThread = new DCToPositionThread(hammerArmMotor, 1440);
    }

    /***
     * Shoots a ring with a power
     * @param power the power with which to shoot a ring [0-1]
     */
    public void shoot(double power){
        if(!shootingThread.isRunning()){
            shootingThread.setPower(power);
            shootingThread.run();
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
    protected void feedMe(){
        // TODO: enable the conveyor to be run again
    }
}
