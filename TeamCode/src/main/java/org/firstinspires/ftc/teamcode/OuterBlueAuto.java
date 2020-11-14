package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.BlueSkyRobot;

@Autonomous(name="Outer Blue Auto", group="Ultimate Goal")
public class OuterBlueAuto extends OpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private BlueSkyRobot blueSky;
    private AutoThread autonomous;


    private class AutoThread extends Thread{

        /***
         * Returns whether this thread is running or not
         * @return true if the thread is running
         */
        public boolean isRunning(){
            return this.isAlive();
        }

        /***
         * Runs the Auto Thread
         */
        @Override
        public void run() {
            runtime.reset();

            while(!blueSky.driveDistance(2006.6, Math.PI / 2)){
                // straight up
                // empty for running the check
            }

            while(!blueSky.driveDistance(777.43, 0)){
                // empty for running the check
            }

            blueSky.shooterShoot();
            runtime.reset();
            while(runtime.milliseconds() < 750){
                // empty to delay time
            }

            runtime.reset();
            while(!blueSky.driveDistance(195.58, 0)){
                blueSky.loaderLower();
                if(runtime.milliseconds() < 300){
                    blueSky.conveyorRun();
                }
            }

            if(runtime.milliseconds() < 300){
                blueSky.conveyorRun();
            }

            blueSky.loaderRaise();
            runtime.reset();
            while(runtime.milliseconds() < 150){
                // empty to delay time
            }

            blueSky.shooterShoot();
            runtime.reset();
            while(runtime.milliseconds() < 750){
                // empty to delay time
            }

            runtime.reset();
            while(!blueSky.driveDistance(195.58, 0)){
                blueSky.loaderLower();
                if(runtime.milliseconds() < 150){
                    blueSky.conveyorRun();
                }
            }

            if(runtime.milliseconds() < 150){
                blueSky.conveyorRun();
            }

            blueSky.loaderRaise();
            runtime.reset();
            while(runtime.milliseconds() < 150){
                // empty to delay time
            }

            blueSky.shooterShoot();
            runtime.reset();
            while(runtime.milliseconds() < 750){
                // empty to delay time
            }

            while(!blueSky.driveDistance(27.4, Math.PI / 2)){
                blueSky.loaderLower();
            }

            //Drive Forward 2006.6mm
            // IF STARTED ON OUTER TAPE MOVE 615.95mm TO THE RIGHT (BLUE) OR LEFT (RED)
            // Move 157.48mm to the right (Blue) or 157.48mm to the right (Red)
            /** Repeat x2
             * Shoot powershot
             * Move 195.58 to the right (blue) or left (red)
             */
            //Shoot powershot 3
            //Drive forward 27.4mm to park
        }
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        this.blueSky = new BlueSkyRobot(hardwareMap, telemetry);
        this.autonomous = new AutoThread();
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // Motor Encoder reset here?

        //Init gyro if we use it?
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        if(!this.autonomous.isRunning()) {
            this.autonomous.start();
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        this.autonomous.interrupt();
    }
}
