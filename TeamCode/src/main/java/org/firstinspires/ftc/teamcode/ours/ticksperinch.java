package org.firstinspires.ftc.teamcode.Ours;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class ticksperinch extends OpMode {
    int average = 0;
    DcMotor trackMotor;
    static Servo bucketServo;
    @Override
    public void init() {
        bucketServo = hardwareMap.servo.get("BucketServo");
        trackMotor = hardwareMap.get(DcMotor.class, "TrackMotor");
        trackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    @Override
    public void loop() {
        trackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        trackMotor.setPower(gamepad1.left_stick_y);
        telemetry.addData("position",trackMotor.getCurrentPosition());
        telemetry.addData("servo position", gamepad1.left_trigger);
        if(trackMotor.getCurrentPosition() > -800) {
            bucketServo.setPosition(.15);
        } else{bucketServo.setPosition(.5);}
        if(gamepad2.a){
            bucketServo.setPosition(.90);
    }
}}
