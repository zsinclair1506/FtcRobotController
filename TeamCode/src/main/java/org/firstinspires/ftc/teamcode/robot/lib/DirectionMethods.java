package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/***
 * @author Zac Sinclair
 * @version 2.0.0
 */

public class DirectionMethods{

public DcMotor frontLeft;
public DcMotor frontRight;
public DcMotor backRight;
public DcMotor backLeft;

     public DirectionMethods(HardwareMap hardwareMap) {
          frontLeft = hardwareMap.dcMotor.get("frontLeft");
          frontRight = hardwareMap.dcMotor.get("frontRight");
          backRight = hardwareMap.dcMotor.get("backRight");
          backLeft = hardwareMap.dcMotor.get("backLeft");
     }

    /***
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
     * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
     public final void sleep(long milliseconds) {
     try   {
               Thread.sleep(milliseconds);
          } catch (InterruptedException e) {
               Thread.currentThread().interrupt();
          }
     }

    /***
     *
     * @param bedtime
     */
    public void forwardDrive(int bedtime) {
          left_drive.setPower(1);
          right_drive.setPower(-1);
          rright_drive.setPower(1);
          rleft_drive.setPower(-1);
          sleep(bedtime);
     }
     
     public void backwardDrive(int bedtime) {
          left_drive.setPower(-1);
          right_drive.setPower(1);
          rright_drive.setPower(-1);
          rleft_drive.setPower(1);
          sleep(bedtime);
     }
     
     public void leftDrive(int bedtime) {
          left_drive.setPower(1);
          right_drive.setPower(-1);
          rright_drive.setPower(-1);
          rleft_drive.setPower(1);
          sleep(bedtime);
     }
     
     public void rightDrive(int bedtime) {
          left_drive.setPower(-1);
          right_drive.setPower(1);
          rright_drive.setPower(1);
          rleft_drive.setPower(-1);
          sleep(bedtime);
     }
     
     public void donutsRight(int bedtime) {
          left_drive.setPower(1);
          right_drive.setPower(1);
          rleft_drive.setPower(1);
          rright_drive.setPower(1);
          sleep(bedtime);
     }
     
     public void donutsLeft(int bedtime) {
          left_drive.setPower(-1);
          right_drive.setPower(-1);
          rleft_drive.setPower(-1);
          rright_drive.setPower(-1);
          sleep(bedtime);
     }
     
     public void stopDrive() {
          left_drive.setPower(0);
          right_drive.setPower(0);
          rleft_drive.setPower(0);
          rright_drive.setPower(0);
     }
     
     public void forwardDriveSlow(int bedtime) {
          left_drive.setPower(0.3);
          right_drive.setPower(-0.3);
          rright_drive.setPower(0.3);
          rleft_drive.setPower(-0.3);
          sleep(bedtime);
     }
     
     public void backwardDriveSlow(int sleeptime) {
          left_drive.setPower(-0.3);
          right_drive.setPower(0.3);
          rright_drive.setPower(-0.3);
          rleft_drive.setPower(0.3);
          sleep(sleeptime);
     }
     
     public void leftDriveSlow(int sleeptime) {
          left_drive.setPower(0.3);
          right_drive.setPower(-0.3);
          rright_drive.setPower(-0.3);
          rleft_drive.setPower(0.3);
          sleep(sleeptime);
     }
     
     public void rightDriveSlow(int sleeptime) {
          left_drive.setPower(-0.3);
          right_drive.setPower(0.3);
          rright_drive.setPower(0.3);
          rleft_drive.setPower(-0.3);
          sleep(sleeptime);
     }
     
     public void donutsRightSlow(int sleeptime) {
          left_drive.setPower(0.3);
          right_drive.setPower(0.3);
          rleft_drive.setPower(0.3);
          rright_drive.setPower(0.3);
          sleep(sleeptime);
     }
     
     public void donutsLeftSlow(int sleeptime) {
          left_drive.setPower(-0.3);
          right_drive.setPower(-0.3);
          rleft_drive.setPower(-0.3);
          rright_drive.setPower(-0.3);
          sleep(sleeptime);
     }
     
     public void leftDriveDless(int sleeptime){
          left_drive.setPower(0.3);
          right_drive.setPower(-0.4);
          rleft_drive.setPower(-0.4);
          rright_drive.setPower(0.3);
          sleep(sleeptime);
     }
     public void rightDriveDless(int sleeptime){
          left_drive.setPower(-0.3);
          right_drive.setPower(0.35);
          rleft_drive.setPower(-0.35);
          rright_drive.setPower(0.3);
          sleep(sleeptime);
     }
}