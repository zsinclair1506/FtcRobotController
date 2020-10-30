package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

/***
 * The class responsible for running and controlling the shooting mechanisms on the robot
 */
public class Shooter extends Mechanism {
    private DcMotor hammerArmMotor;
    private ShooterThread shootingThread;

    private class ShooterThread extends Thread{
        // As a note, I'm assuming Telemetry is not thread safe so not using it here
        private DcMotor shooterMotor;
        private double motorPower;

        protected ShooterThread (DcMotor shooterMotor){
            this.shooterMotor = shooterMotor;
            init();
        }

        private void init(){
            hammerArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hammerArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        protected boolean isRunning(){
            return this.isAlive();
        }

        @Override
        public void run() {
            if (hammerArmMotor.getCurrentPosition() < 1440) {
                hammerArmMotor.setTargetPosition(1440);
                hammerArmMotor.setPower(motorPower);
            }else {
                hammerArmMotor.setPower(0);
            }

            feedMe();
        }

        public void setPower(double power) {
            this.motorPower = power;
        }
    }

    /***
     * Shooter constructor
     * @param map hardware map of the robot
     */
    public Shooter(HardwareMap map, Telemetry telemetry){
        super(telemetry);
        this.hammerArmMotor = map.get(DcMotor.class, MotorMap.SHOOTER_DC.getMotorName());
        this.shootingThread = new ShooterThread(hammerArmMotor);
    }

    /***
     * Shoots a ring with a @power
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
