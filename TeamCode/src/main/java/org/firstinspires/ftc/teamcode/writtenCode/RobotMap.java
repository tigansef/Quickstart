package org.firstinspires.ftc.teamcode.writtenCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotMap {

    public DcMotorEx intake,transfer;

    public Servo hood;
    public Servo turret1,turret2;
    public Servo stopper;
    public Servo pto;
    public Servo forbar1,forbar2;


    public Servo brake1,brake2;

    public RobotMap(HardwareMap Init)
    {
        intake = Init.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer = Init.get(DcMotorEx.class, "transfer");
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        hood = Init.get(Servo.class, "hood");
        turret1 = Init.get(Servo.class, "turret1");
        turret2 = Init.get(Servo.class, "turret2");
        stopper = Init.get(Servo.class, "stopper");
        brake1 = Init.get(Servo.class, "brake1");
        brake2 = Init.get(Servo.class, "brake2");
        pto = Init.get(Servo.class, "pto");
        forbar1 = Init.get(Servo.class, "forbar1");
        forbar2 = Init.get(Servo.class, "forbar2");



    }
}
