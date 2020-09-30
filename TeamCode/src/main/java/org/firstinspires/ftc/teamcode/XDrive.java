package org.firstinspires.ftc.teamcode;

// Again, I copied the imports, originally, these were in RobotTest4
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


/**
 * Version Start Date and Time
 * Format: Version - Date Time Timezone Team Member
 *
 * Version 1.0 - 2020-10-1 3:05AM AEST Zac S.
 */

/**
 * Version Finish Date and Time
 * Format: Version - Date Time Timezone Team Member
 *
 * Version 1.0 - 2020-10-1 4:02AM AEST Zac S.
 */

@TeleOp (name="XDrive-Beta1", group="XDrive Beta")

public class XDrive extends Drivebase {
    public XDrive(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    // I know this is how Minecraft Coordinates work so I assume it carries over to real life
    float xAxisPower; // East/ West
    float zAxisPower; // North/ South

    @Override
    void drive() {
        telemetry.addData("Function", "Driving");
        // I understand that I didn't consider putting left-right and forward-back controls into one method
        if (gamepad1.left_stick_y != 0) {
            zAxisPower = gamepad1.left_stick_y;
            frontLeft.setPower(zAxisPower);
            frontRight.setPower(-zAxisPower);
            backLeft.setPower(zAxisPower);
            backRight.setPower(-zAxisPower);
        } else if (gamepad1.left_stick_x != 0) {
            xAxisPower = gamepad1.left_stick_x;
            frontLeft.setPower(xAxisPower);
            frontRight.setPower(xAxisPower);
            backLeft.setPower(-xAxisPower);
            backRight.setPower(-xAxisPower);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    } // I am not sure if either method is going to work as intended, or if they are around the wrong way
    // I can't test if this is the case, because I don't have the materials (Expansion Hub, Battery and 1 DC Motor)

    @Override
    void rotate() {
        telemetry.addData("Function", "Rotating");
        if (gamepad1.right_stick_x != 0) {
            frontLeft.setPower(xAxisPower);
            frontRight.setPower(xAxisPower);
            backLeft.setPower(xAxisPower);
            backRight.setPower(xAxisPower);
        }
    }
}