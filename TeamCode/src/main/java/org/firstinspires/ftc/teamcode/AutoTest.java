package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.BlueSkyRobot;
import org.firstinspires.ftc.teamcode.robot.lib.GamepadWrapper;
import org.firstinspires.ftc.teamcode.robot.mapping.GamepadButtons;

@Autonomous(name="Auto Test", group="Robot Test")
public class AutoTest extends OpMode{

        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private BlueSkyRobot blueSky;

        // Set to print telemetry data to the phone
        private boolean debug = false;

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
             * 100rpm = 10.47 rad/s
             * wheel radius = 4in
             * Forward Velocity = 10.47 x 4 x √2 = 59.227
             * TODO: Double Check Accuracy
             */

            //Drive Forward X Distance

            /** Repeat x2
            * Shoot powershot
            * Move Left
            */
            //Shoot powershot 3
            //Drive forward to park
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
