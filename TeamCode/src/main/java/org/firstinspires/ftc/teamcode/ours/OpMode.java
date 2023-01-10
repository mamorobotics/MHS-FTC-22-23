package org.firstinspires.ftc.teamcode.ours;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

@ com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class OpMode extends LinearOpMode {
    static DcMotor FL, BL, FR, BR;

    static Servo arm1, arm2, clawControlServo, clawServo;

    static double speed = 1;

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
                clawServo.setPosition(.25);
            } else {
                clawServo.setPosition(0);
            }

            arm1.setPosition(arm1.getPosition() + (gamepad2.left_stick_y / 100));
            arm2.setPosition(arm2.getPosition() + (gamepad2.right_stick_y / 100));

            move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed);
        }
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
