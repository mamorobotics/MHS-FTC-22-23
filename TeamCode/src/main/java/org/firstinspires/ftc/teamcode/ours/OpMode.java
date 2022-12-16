package org.firstinspires.ftc.teamcode.ours;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@ com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class OpMode extends LinearOpMode {
    static DcMotor FL, BL, FR, BR;

    static DcMotor LM;
    /*static Servo clawServo;
    static Servo clawControlServo;*/

    static double speed = 1;

    //static boolean aPressed, clawToggle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BL = hardwareMap.get(DcMotor.class, "leftRear");
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        BR = hardwareMap.get(DcMotor.class, "rightRear");

        LM = hardwareMap.get(DcMotor.class, "liftMotor");
        /*clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawControlServo = hardwareMap.get(Servo.class, "clawControlServo");*/

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        
        while (opModeIsActive()){
            //telemetry.addData("Claw Toggle", clawToggle);
            telemetry.addData("X", -gamepad1.left_stick_y);
            telemetry.addData("Y", gamepad1.left_stick_x);
            telemetry.update();

            /*if(gamepad2.a) {
                if (!aPressed) {
                    //Toggled on
                    clawServo.setPosition(90);
                }
                aPressed = true;
            } else {
                //Toggled off
                aPressed = false;
                clawServo.setPosition(0);
            }*/

            double[] angles = calcArmAngles(4, 4, 4);

            if(LM.getCurrentPosition() > 0 && LM.getCurrentPosition() < 100) {
                LM.setPower(gamepad2.left_stick_y);
            }

            move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed);
        }
    }

    public static double[] calcArmAngles(double x, double y, double length){
        double[] out = new double[2];

        out[1] = -Math.acos((Math.pow(x, 2) + Math.pow(y, 2) - Math.pow(length, 2) - Math.pow(length, 2)) / (2 * length * length));
        out[0] = Math.atan(y / x) - Math.atan((length * Math.sin(out[1])) / (length + length * Math.cos(out[1])));
        return out;
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