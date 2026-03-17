package org.firstinspires.ftc.teamcode.writtenCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp(name = "Two Servo Example")
public class servoTest extends LinearOpMode {

    public static double pos=0.5;
    public static Servo servo1;
    public static Servo servo2;

    @Override
    public void runOpMode() {
        servo1 = hardwareMap.get(Servo.class, "turret2");
        //servo2 = hardwareM
        // ap.get(Servo.class, "forbar2");

        servo1.setPosition(pos);
       // servo2.setPosition(pos);

        telemetry.addData("Status", "Initialized - Servos set to 0.5");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            servo1.setPosition(pos);
            //servo2.setPosition(pos);

            telemetry.update();


        }
    }
}