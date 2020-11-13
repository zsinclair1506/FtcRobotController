package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.BlueSkyRobot;
import org.firstinspires.ftc.teamcode.robot.mapping.GamepadButtons;
import org.firstinspires.ftc.teamcode.robot.lib.GamepadWrapper;

import java.util.HashMap;

@TeleOp(name="BlueSky Test", group="Ultimate Goal")
public class UltimateGoalOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private BlueSkyRobot blueSky;
    private GamepadWrapper driveGamepad, operatorGamepad;
    private HashMap<GamepadButtons, ElapsedTime> debounceTimers = new HashMap<>();
    private HashMap<GamepadButtons, Boolean> debounce = new HashMap<>();
    private static final double DEBOUNCE_TIME_MS = 250;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        this.blueSky = new BlueSkyRobot(hardwareMap, telemetry);
        this.driveGamepad = new GamepadWrapper(gamepad1);
        this.operatorGamepad = new GamepadWrapper(gamepad2);

        this.debounceTimers.put(GamepadButtons.INTAKE_SWITCH_POSITIONS, new ElapsedTime());
        this.debounceTimers.put(GamepadButtons.INTAKE_CYCLE, new ElapsedTime());
        this.debounceTimers.put(GamepadButtons.INTAKE_STOP, new ElapsedTime());
        this.debounceTimers.put(GamepadButtons.SHOOTER_FEED_ME, new ElapsedTime());
        this.debounceTimers.put(GamepadButtons.SHOOTER_SHOOT, new ElapsedTime());
        this.debounceTimers.put(GamepadButtons.FEEDER_DOWN, new ElapsedTime());
        this.debounceTimers.put(GamepadButtons.FEEDER_UP, new ElapsedTime());
        this.debounce.put(GamepadButtons.INTAKE_SWITCH_POSITIONS, false);
        this.debounce.put(GamepadButtons.INTAKE_CYCLE, false);
        this.debounce.put(GamepadButtons.INTAKE_STOP, false);
        this.debounce.put(GamepadButtons.SHOOTER_FEED_ME, false);
        this.debounce.put(GamepadButtons.SHOOTER_SHOOT, false);
        this.debounce.put(GamepadButtons.FEEDER_DOWN, false);
        this.debounce.put(GamepadButtons.FEEDER_UP, false);

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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        blueSky.setStrafe(driveGamepad.getStickVector(GamepadButtons.ROBOT_STRAFE.getButtonName()));
        blueSky.setRotate(driveGamepad.getStickVector(GamepadButtons.ROBOT_ROTATE.getButtonName()));
        blueSky.drive();
//
        if(driveGamepad.getTriggerBool(GamepadButtons.SHOOTER_SHOOT.getButtonName())){
            // debounce
            if(!this.debounce.get(GamepadButtons.SHOOTER_SHOOT)){
                this.debounce.put(GamepadButtons.SHOOTER_SHOOT, true);
                this.debounceTimers.get(GamepadButtons.SHOOTER_SHOOT).reset();
                blueSky.shooterShoot();
            }
        }

        if(driveGamepad.getButton(GamepadButtons.SHOOTER_FEED_ME.getButtonName())){
            // debounce
            if(!this.debounce.get(GamepadButtons.SHOOTER_FEED_ME)){
                this.debounce.put(GamepadButtons.SHOOTER_FEED_ME, true);
                this.debounceTimers.get(GamepadButtons.SHOOTER_FEED_ME).reset();
                blueSky.shooterFeedMe();
            }
        }

        if (driveGamepad.getButton(GamepadButtons.FEEDER_UP.getButtonName())){
            // debounce
            if(!this.debounce.get(GamepadButtons.FEEDER_UP)){
                this.debounce.put(GamepadButtons.FEEDER_UP, true);
                this.debounceTimers.get(GamepadButtons.FEEDER_UP).reset();
                blueSky.loaderRaise();
            }
        }

        if (driveGamepad.getButton(GamepadButtons.FEEDER_DOWN.getButtonName())){
            // debounce
            if(!this.debounce.get(GamepadButtons.FEEDER_DOWN)){
                this.debounce.put(GamepadButtons.FEEDER_DOWN, true);
                this.debounceTimers.get(GamepadButtons.FEEDER_DOWN).reset();
                blueSky.loaderLower();
            }
        }

        if(operatorGamepad.getTriggerBool(GamepadButtons.CONVEYOR_RUN.getButtonName())){
            blueSky.conveyorRun();
        }
        else if(operatorGamepad.getTriggerBool(GamepadButtons.CONVEYOR_RUN.getButtonName())){
            blueSky.conveyorStop();
        }

        if(operatorGamepad.getButton(GamepadButtons.INTAKE_LIFT.getButtonName())){
            blueSky.intakeLift();
        }
        else if(operatorGamepad.getButton(GamepadButtons.INTAKE_LOWER.getButtonName())){
            blueSky.intakeLower();
        }
        else{
            blueSky.intakeStop();
        }

        if(operatorGamepad.getButton(GamepadButtons.INTAKE_GRAB.getButtonName())){
            blueSky.intakeGrab();
        }
        else if(operatorGamepad.getButton(GamepadButtons.INTAKE_RELEASE.getButtonName())){
            blueSky.intakeRelease();
        }

        if(operatorGamepad.getTriggerBool(GamepadButtons.INTAKE_SWITCH_POSITIONS.getButtonName())){
            // debounce
            if(!this.debounce.get(GamepadButtons.INTAKE_SWITCH_POSITIONS)){
                this.debounce.put(GamepadButtons.INTAKE_SWITCH_POSITIONS, true);
                this.debounceTimers.get(GamepadButtons.INTAKE_SWITCH_POSITIONS).reset();
                blueSky.intakeRotate();
            }
        }

        if(operatorGamepad.getButton(GamepadButtons.INTAKE_CYCLE.getButtonName())){
            // debounce
            if(!this.debounce.get(GamepadButtons.INTAKE_CYCLE)){
                this.debounce.put(GamepadButtons.INTAKE_CYCLE, true);
                this.debounceTimers.get(GamepadButtons.INTAKE_CYCLE).reset();
                blueSky.intakeCycle();
            }
        }

        if(operatorGamepad.getButton(GamepadButtons.INTAKE_STOP.getButtonName())){
            // debounce
            if(!this.debounce.get(GamepadButtons.INTAKE_STOP)){
                this.debounce.put(GamepadButtons.INTAKE_STOP, true);
                this.debounceTimers.get(GamepadButtons.INTAKE_STOP).reset();
                blueSky.intakeCancel();
            }
        }

        for(GamepadButtons button : this.debounce.keySet()) {
            if (debounceTimers.get(button).milliseconds() > DEBOUNCE_TIME_MS) {
                this.debounce.put(button, false);
            }
        }
    }

    /***
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
