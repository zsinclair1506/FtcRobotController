package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;



public class AttachmentMethods{

public Servo pservo1; // pservo = pivot servo
public Servo pservo2;
public CRServo pservo3;
public CRServo pservo4;
public CRServo lservo1; // lservo = lift servo
public CRServo lservo2;
public CRServo iservo0; // iservo = intake servo
public CRServo iservo1;
public CRServo iservo2;
public CRServo iservo3;
public Servo cservo1; //cservo = christmas tree servo
public Servo cservo2; 
  /**
   * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
   * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
   *
   * @param milliseconds amount of time to sleep, in milliseconds
   * @see Thread#sleep(long)
   */
     public AttachmentMethods(HardwareMap hardwareMap){
          iservo0 = hardwareMap.get(CRServo.class,"servo0");
          iservo1 = hardwareMap.get(CRServo.class,"servo1");
          iservo2 = hardwareMap.get(CRServo.class,"servo2");
          iservo3 = hardwareMap.get(CRServo.class,"servo3");
          
          pservo1 = hardwareMap.get(Servo.class,"pservo1");
          pservo2 = hardwareMap.get(Servo.class,"pservo2");
          pservo3 = hardwareMap.get(CRServo.class, "pservo3");
          pservo4 = hardwareMap.get(CRServo.class, "pservo4");
          lservo1 = hardwareMap.get(CRServo.class, "lservo1");
          lservo2 = hardwareMap.get(CRServo.class, "lservo2");
          
          cservo1 = hardwareMap.get(Servo.class, "cservo1");
          cservo2 = hardwareMap.get(Servo.class, "cservo2");
     }
     public final void sleep(long milliseconds) {
     try   {
               Thread.sleep(milliseconds);
          } catch (InterruptedException e) {
               Thread.currentThread().interrupt();
          }
     }
     public void flipUp(int bedtime){
        pservo1.setPosition(1.0);
        pservo2.setPosition(0.0);
          sleep(bedtime);
     }
     public void flipDown(int bedtime){
        pservo1.setPosition(0.0);
        pservo2.setPosition(1.0);
          sleep(bedtime);
     }
     public void flipUpRack(int bedtime){
          pservo3.setPower(1);
          pservo4.setPower(-1);
          sleep(bedtime);
     }
     public void flipDownRack(int bedtime){
          pservo3.setPower(-1);
          pservo4.setPower(1);
          sleep(bedtime);
     }
     public void liftOne(int bedtime){
          lservo1.setPower(1);
          sleep(bedtime);
     }
     public void liftTwo(int bedtime){
          lservo2.setPower(1);
          sleep(bedtime);
     }
     
     public void treesUp(int bedtime){
          cservo1.setPosition(0);
          cservo2.setPosition(1);
          sleep(bedtime);
     }
     public void treesDown(int bedtime){
          cservo1.setPosition(1);
          cservo2.setPosition(0);
          sleep(bedtime);
     }
}
