package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.BlueSkyRobot;

@Autonomous(name="Auto Test", group="Robot Test")
public class AutoTest extends OpMode{
        private ElapsedTime runtime = new ElapsedTime();
        private BlueSkyRobot blueSky;


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


            /**
             * Wheel Circumference = 12.57in
             * Distance to edge of scoring zone = 80in
             *
             * Motor Calcs
             * wheel radius = 2in
             * Forward Velocity = 10.47... x 2 x √2 = 29.619219598...
             * TODO: Double Check Accuracy
             */

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

        }
    }
