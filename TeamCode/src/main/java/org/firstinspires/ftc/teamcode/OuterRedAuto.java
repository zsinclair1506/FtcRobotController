package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.BlueSkyRobot;

@Autonomous(name="Outer Red Auto", group="Ultimate Goal")
public class OuterRedAuto extends OpMode{
        private ElapsedTime runtime = new ElapsedTime();
        private BlueSkyRobot blueSky;
        // This variable stores the current 'state' or instruction that the robot is undertaking
        // It is made a byte because floating points are not necessary and it seems unlikely
        // That there will be more than 128 phases that the robot has to undergo in 30 seconds
        private byte state = 1;  


        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init() {
            this.blueSky = new BlueSkyRobot(hardwareMap, telemetry);

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
            runtime.reset();

            /*** while(!this.blueSky.driveDistance(2006.6, Math.PI / 2)){
                // straight up
                // empty for running the check
            }

            while(!this.blueSky.driveDistance(619.95, Math.PI)){
                // empty for running the check
            }

            this.blueSky.shooterShoot();
            runtime.reset();
            while(runtime.milliseconds() < 750){
                // empty to delay time
            }

            runtime.reset();
            while(!this.blueSky.driveDistance(195.58, Math.PI)){
                this.blueSky.loaderLower();
                if(runtime.milliseconds() < 300){
                    this.blueSky.conveyorRun();
                }
            }

            if(runtime.milliseconds() < 300){
                this.blueSky.conveyorRun();
            }

            this.blueSky.loaderRaise();
            runtime.reset();
            while(runtime.milliseconds() < 150){
                // empty to delay time
            }

            this.blueSky.shooterShoot();
            runtime.reset();
            while(runtime.milliseconds() < 750){
                // empty to delay time
            }

            runtime.reset();
            while(!this.blueSky.driveDistance(195.58, Math.PI)){
                this.blueSky.loaderLower();
                if(runtime.milliseconds() < 150){
                    this.blueSky.conveyorRun();
                }
            }

            if(runtime.milliseconds() < 150){
                this.blueSky.conveyorRun();
            }

            this.blueSky.loaderRaise();
            runtime.reset();
            while(runtime.milliseconds() < 150){
                // empty to delay time
            }

            this.blueSky.shooterShoot();
            runtime.reset();
            while(runtime.milliseconds() < 750){
                // empty to delay time
            }

            while(!this.blueSky.driveDistance(27.4, Math.PI / 2)){
                this.blueSky.loaderLower();
            } */

            //Drive Forward 2006.6mm
            // IF STARTED ON OUTER TAPE MOVE 615.95mm TO THE RIGHT (BLUE) OR LEFT (RED)
            // Move 157.48mm to the right (Blue) or 157.48mm to the left (Red)
            /** Repeat x2
            * Shoot powershot
            * Move 195.58 to the right (blue) or left (red)
            */
            //Shoot powershot 3
            //Drive forward 27.4mm to park
        }

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
        @Override
        public void loop() {
            switch (state) {
                case 1:
                    state = this.blueSky.driveDistance(2006.6, Math.PI/2, state);
                    break;
                case 2:
                    state = this.blueSky.driveDistance(615.95, Math.PI, state);
                    break;
                case 3:
                    state = this.blueSky.driveDistance(157.48, Math.PI, state);
                    break;
                case 4:
                    state = this.blueSky.driveDistance(195.58, Math.PI, state);
                    break;
                case 5:
                    state = this.blueSky.driveDistance(195.58, Math.PI, state);    
                    break;
                case 6:
                    state = this.blueSky.driveDistance(27.4, Math.PI/2, state);
                default:
                    this.blueSky.setStrafe(0,0);
                    break;
            }
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
            this.blueSky.setStrafe(0,0);
        }
    }
