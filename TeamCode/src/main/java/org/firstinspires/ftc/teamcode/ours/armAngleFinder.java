package org.firstinspires.ftc.teamcode.ours;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

@ com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "AngleFinder")
public class armAngleFinder extends LinearOpMode{
    static Servo l, u;
    double a1, a2 = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        l = hardwareMap.get(Servo.class, "armServo1");
        u = hardwareMap.get(Servo.class, "armServo2");

        waitForStart();

        while(opModeIsActive())
        {
            l.setPosition(a1);
            u.setPosition(a2);

            if(gamepad1.dpad_up){
                a1+=0.005;
            }
            if(gamepad1.dpad_down){
                a1-=0.005;
            }
            if(gamepad1.dpad_left){
                a2+=0.005;
            }
            if(gamepad1.dpad_right){
                a2-=0.005;
            }
            telemetry.addData("a1: ", a1);
            telemetry.addData("a2", a2);
            telemetry.update();
        }
    }
}
