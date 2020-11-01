package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.BlueSkyRobot;
import org.firstinspires.ftc.teamcode.robot.mapping.GamepadButtons;
import org.firstinspires.ftc.teamcode.robot.lib.GamepadWrapper;

@TeleOp(name="Whack Motor Test", group="Robot Test")
public class UltimateGoalOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private BlueSkyRobot blueSky;
    private GamepadWrapper driveGamepad, operatorGamepad;

    // Set to print telemetry data to the phone
    private boolean debug = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        this.blueSky = new BlueSkyRobot(hardwareMap, telemetry);
        this.driveGamepad = new GamepadWrapper(gamepad1, telemetry);
        this.operatorGamepad = new GamepadWrapper(gamepad2, telemetry);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        blueSky.setStrafe(driveGamepad.getStickVector(GamepadButtons.ROBOT_DRIVE.getButtonName()));
        blueSky.setRotate(driveGamepad.getStickVector(GamepadButtons.ROBOT_ROTATE.getButtonName()));
        blueSky.drive();

        blueSky.clawOpen(operatorGamepad.getButton(GamepadButtons.GRIPPER_OPEN.getButtonName()));

        blueSky.clawClose(operatorGamepad.getButton(GamepadButtons.GRIPPER_CLOSE.getButtonName()));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
