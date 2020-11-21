package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.BlueSkyRobot;
import org.firstinspires.ftc.teamcode.robot.mapping.GamepadButtonMap;
import org.firstinspires.ftc.teamcode.robot.lib.GamepadWrapper;

@TeleOp(name="BlueSky TeleOp", group="Ultimate Goal")
public class UltimateGoalOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private BlueSkyRobot blueSky;
    private GamepadWrapper driveGamepad, operatorGamepad;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        this.blueSky = new BlueSkyRobot(hardwareMap, telemetry);
        this.driveGamepad = new GamepadWrapper(gamepad1, 1);
        this.operatorGamepad = new GamepadWrapper(gamepad2, 2);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        this.runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        this.blueSky.setStrafe(driveGamepad.getStickVector(GamepadButtonMap.DriverGamepad.ROBOT_STRAFE));
        this.blueSky.setRotate(driveGamepad.getStickVector(GamepadButtonMap.DriverGamepad.ROBOT_ROTATE));
        this.blueSky.drive();

        if(driveGamepad.getTriggerBool(GamepadButtonMap.DriverGamepad.SHOOTER_SHOOT)){
            this.blueSky.shooterShoot();
        }

        if(driveGamepad.getButton(GamepadButtonMap.DriverGamepad.SHOOTER_FEED_ME)){
            this.blueSky.shooterFeedMe();
        }

        if (driveGamepad.getButton(GamepadButtonMap.DriverGamepad.FEEDER_UP)){
            this.blueSky.loaderRaise();
        }

        if (driveGamepad.getButton(GamepadButtonMap.DriverGamepad.FEEDER_DOWN)){
            this.blueSky.loaderLower();
        }

        if(operatorGamepad.getTriggerBool(GamepadButtonMap.OperatorGamepad.INTAKE_SWITCH_POSITIONS)){
            this.blueSky.intakeRotate();
        }

        if(operatorGamepad.getButton(GamepadButtonMap.OperatorGamepad.INTAKE_CYCLE)){
            this.blueSky.intakeCycle();
        }

        if(operatorGamepad.getButton(GamepadButtonMap.OperatorGamepad.INTAKE_STOP)){
            this.blueSky.intakeCancel();
        }

        if(operatorGamepad.getTriggerBool(GamepadButtonMap.OperatorGamepad.CONVEYOR_RUN)){
            this.blueSky.conveyorRun();
        }
        else if(operatorGamepad.getTriggerBool(GamepadButtonMap.OperatorGamepad.CONVEYOR_RUN)){
//            this.blueSky.conveyorStop();
        }

        if(operatorGamepad.getButton(GamepadButtonMap.OperatorGamepad.INTAKE_LIFT)){
            this.blueSky.intakeLift();
        }
        else if(operatorGamepad.getButton(GamepadButtonMap.OperatorGamepad.INTAKE_LOWER)){
            this.blueSky.intakeLower();
        }
        else{
//            this.blueSky.intakeStop();
        }

        if(operatorGamepad.getButton(GamepadButtonMap.OperatorGamepad.INTAKE_GRAB)){
            this.blueSky.intakeGrab();
        }
        else if(operatorGamepad.getButton(GamepadButtonMap.OperatorGamepad.INTAKE_RELEASE)){
//            this.blueSky.intakeRelease();
        }
    }

    /***
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
//        this.blueSky.stop();
    }
}
