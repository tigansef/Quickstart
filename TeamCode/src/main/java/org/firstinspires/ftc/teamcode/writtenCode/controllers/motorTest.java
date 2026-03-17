package org.firstinspires.ftc.teamcode.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Two Servo Example")
public class motorTest extends LinearOpMode {

    public static double pos;
    public static DcMotorEx transfer;

    @Override
    public void runOpMode() {
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        //servo2 = hardwareMap.get(Servo.class, "forbar2");

       // servo2.setPosition(pos);

        telemetry.addData("Status", "Initialized - Servos set to 0.5");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
transfer.setPower(1);            //servo2.setPosition(pos);

            telemetry.update();


        }
    }
}