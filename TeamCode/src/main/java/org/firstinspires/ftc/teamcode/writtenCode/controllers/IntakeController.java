package org.firstinspires.ftc.teamcode.writtenCode.controllers;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.writtenCode.RobotMap;
@Config

@Configurable
public class IntakeController {
    public enum IntakeStatus{
        COLLECT,
        REVERSE,
        TRANSFER,
        OFF;
    }
    public IntakeStatus currentStatus= IntakeStatus.OFF;
    public IntakeStatus previousStatus=null;

    public static double collectpow = 1;
    public static double transferpow = 0.8;

    public DcMotorEx intake = null;

    public IntakeController(RobotMap robot){this.intake=robot.intake;}
    public void update()
    {
        if(currentStatus!=previousStatus){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case COLLECT:
                {
                    this.intake.setPower(collectpow);
                    break;
                }
                case TRANSFER:
                {
                    this.intake.setPower(transferpow);
                    break;
                }
                case OFF:
                {
                    this.intake.setPower(0);
                    break;
                }
                case REVERSE:
                {
                    this.intake.setPower(-1);
                    break;
                }
            }
        }
    }
}

