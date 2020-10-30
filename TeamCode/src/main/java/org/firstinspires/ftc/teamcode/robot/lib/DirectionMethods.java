package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



public class DirectionMethods{

public DcMotor left_drive;
public DcMotor right_drive;
public DcMotor rright_drive;
public DcMotor rleft_drive;
  /**
   * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
   * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
   *
   * @param milliseconds amount of time to sleep, in milliseconds
   * @see Thread#sleep(long)
   */
     public DirectionMethods(HardwareMap hardwareMap) {
          left_drive = hardwareMap.dcMotor.get("left_drive");
          right_drive = hardwareMap.dcMotor.get("right_drive");
          rright_drive = hardwareMap.dcMotor.get("rright_drive");
          rleft_drive = hardwareMap.dcMotor.get("rleft_drive");
     }
     
     public final void sleep(long milliseconds) {
     try   {
               Thread.sleep(milliseconds);
          } catch (InterruptedException e) {
               Thread.currentThread().interrupt();
          }
     }
     
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
     
     public void backwardDriveSlow(int bedtime) {
          left_drive.setPower(-0.3);
          right_drive.setPower(0.3);
          rright_drive.setPower(-0.3);
          rleft_drive.setPower(0.3);
          sleep(bedtime);
     }
     
     public void leftDriveSlow(int bedtime) {
          left_drive.setPower(0.3);
          right_drive.setPower(-0.3);
          rright_drive.setPower(-0.3);
          rleft_drive.setPower(0.3);
          sleep(bedtime);
     }
     
     public void rightDriveSlow(int bedtime) {
          left_drive.setPower(-0.3);
          right_drive.setPower(0.3);
          rright_drive.setPower(0.3);
          rleft_drive.setPower(-0.3);
          sleep(bedtime);
     }
     
     public void donutsRightSlow(int bedtime) {
          left_drive.setPower(0.3);
          right_drive.setPower(0.3);
          rleft_drive.setPower(0.3);
          rright_drive.setPower(0.3);
          sleep(bedtime);
     }
     
     public void donutsLeftSlow(int bedtime) {
          left_drive.setPower(-0.3);
          right_drive.setPower(-0.3);
          rleft_drive.setPower(-0.3);
          rright_drive.setPower(-0.3);
          sleep(bedtime);
     }
     
     public void leftDriveDless(int bedtime){
          left_drive.setPower(0.3);
          right_drive.setPower(-0.4);
          rleft_drive.setPower(-0.4);
          rright_drive.setPower(0.3);
          sleep(bedtime);
     }
     public void rightDriveDless(int bedtime){
          left_drive.setPower(-0.3);
          right_drive.setPower(0.35);
          rleft_drive.setPower(-0.35);
          rright_drive.setPower(0.3);
          sleep(bedtime);
     }
}