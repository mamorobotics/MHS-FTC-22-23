package org.firstinspires.ftc.teamcode.ours;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

@ com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class OpMode extends LinearOpMode {
    static DcMotor FL, BL, FR, BR;

    static Servo arm1, arm2, clawControlServo, clawServo;

    static double speed = 0.65;

    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BL = hardwareMap.get(DcMotor.class, "leftRear");
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        BR = hardwareMap.get(DcMotor.class, "rightRear");

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawControlServo = hardwareMap.get(Servo.class, "clawControlServo");

        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        
        while (opModeIsActive()){
            telemetry.addData("X", -gamepad1.left_stick_y);
            telemetry.addData("Y", gamepad1.left_stick_x);

            telemetry.addData("Arm 1 Angle", arm1.getPosition());
            telemetry.addData("Arm 2 Angle", arm2.getPosition());


            telemetry.update();

            if(gamepad2.right_trigger > 0.1) {
                clawServo.setPosition(.24);
            } else {
                clawServo.setPosition(0);
            }


            //Pickup


            if(gamepad2.dpad_right) {
                double starttime = this.getRuntime();
                while ( this.getRuntime() - starttime < 1.5) {
                    clawControlServo.setPosition(0.1);
                    move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed);
                }
                    arm2.setPosition(0);
                    arm1.setPosition(0.43);

            }
            //High Pole
            if(gamepad2.dpad_up) {
                arm1.setPosition(0);
                arm2.setPosition(0.3);
                clawControlServo.setPosition(0.7);
            }
            //Medium Pole
            if(gamepad2.dpad_left) {
                arm1.setPosition(0.05);
                arm2.setPosition(0);
                clawControlServo.setPosition(0.38);
            }
            //Low Pole
            if(gamepad2.dpad_down) {
                arm1.setPosition(0.2);
                arm2.setPosition(0);
                clawControlServo.setPosition(0.25);
            }
            if(gamepad2.x) {
                arm1.setPosition(0);
                arm2.setPosition(0);
                clawControlServo.setPosition(0);
            }
            if(gamepad2.y){
                arm1.setPosition(0.65);
                arm2.setPosition(0.55);
                clawControlServo.setPosition(0.4);
            }
            if(gamepad2.b)
            {
                clawControlServo.setPosition(0.15);
            }


            move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed);
        }
    }

    public static double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    private static void move(double x, double y, double r, double speed){
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        double frontLeftPower = (y + x + r) * speed / denominator;
        double backLeftPower = (y - x + r) * speed / denominator;
        double frontRightPower = (y - x - r) * speed / denominator;
        double backRightPower = (y + x - r) * speed / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);
    }
}
