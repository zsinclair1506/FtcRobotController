package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.BlueSkyRobot;
import org.firstinspires.ftc.teamcode.robot.GamepadButtons;
import org.firstinspires.ftc.teamcode.robot.lib.GamepadWrapper;
import org.firstinspires.ftc.teamcode.robot.lib.Vector;


@TeleOp(name="Default TeleOp", group="Final RunModes")
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
        telemetry.addData("Status", "Initialized");

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
        telemetry.addData("Button", GamepadButtons.ROBOT_DRIVE.getButtonName());
        telemetry.addData("Mag",
                driveGamepad.getStickVector(GamepadButtons.ROBOT_DRIVE.getButtonName()).getMagnitude());
        telemetry.addData("Angle",
                driveGamepad.getStickVector(GamepadButtons.ROBOT_DRIVE.getButtonName())
                        .getAngleBetween(Vector.X_2));
        blueSky.drive(driveGamepad.getStickVector(GamepadButtons.ROBOT_DRIVE.getButtonName()));
        telemetry.update();
        /**
        if (driveGamepad.getTrigger(GamepadButtons.SHOOTER_SHOOT.getButtonName()) > 0.50) {
            blueSky.shooterShoot();
        }
        if (operatorGamepad.getTrigger(GamepadButtons.CONVEYOR_RUN.getButtonName()) > 0.2){
            blueSky.conveyorRun();
        }
        */
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}