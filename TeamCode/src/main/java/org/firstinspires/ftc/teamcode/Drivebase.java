package org.firstinspires.ftc.teamcode;

// Imports! YAY! I just copied these from the old direction method class from SkyStone
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Drivebase {

    //The motors, again copied and renamed :)
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor backLeft;

    // My favourite, hardware maps, any and all errors are on the phones or expansion hubs, not here, becuase I KNOW this works
    // For the sake of simplicity, I am listing these motors from 0-3, and will include a diagram too
    public Drivebase(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.dcMotor.get("frontleft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
    }

    // abstract methods
    abstract void drive();
    abstract void rotate();

    //concrete method
    void motorStats() {
        // I forgot what I need to put here and don't really care right now [2020-10-1 2:50 AEST]
        // I remembered what to put here - TELEMETRY, still don't care though [2020-10-1 3:11AM AEST]
    }

}
