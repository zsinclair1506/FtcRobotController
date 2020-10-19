package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;


@TeleOp(name="Whack Motor Test C", group="Robot Test")
public class UltimateGoalOpModeC extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor testMotor1, testMotor2, whackMotor1;
    private CRServo conveyor1;
    private int encoderTarget = 1440;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        testMotor1 = hardwareMap.get(DcMotor.class, "testMotor1");
        testMotor2 = hardwareMap.get(DcMotor.class, "testMotor2");
        whackMotor1 = hardwareMap.get(DcMotor.class,"whackMotor1");

        conveyor1 = hardwareMap.get(CRServo.class, "conveyor1");

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
        double power = gamepad1.right_stick_y;

        conveyor1.setPower(1);

        if(power > 1){
            power = 1;
        }
        else if(power < -1){
            power = -1;
        }

        testMotor1.setPower(power);
        testMotor2.setPower(-power);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}

