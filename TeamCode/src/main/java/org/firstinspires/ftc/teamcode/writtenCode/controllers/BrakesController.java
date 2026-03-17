package org.firstinspires.ftc.teamcode.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.writtenCode.RobotMap;

@Config

@Configurable
public class BrakesController {
    public enum ForbarScoreStatus{
        INIT,
        ON;
    }
    public ForbarScoreStatus currentStatus = ForbarScoreStatus.INIT;
    public ForbarScoreStatus previousStatus=null;
    public static double init_position=0;
    public static double on_position=0;


    public Servo brake1 = null;
    public Servo brake2 = null;

    public BrakesController(RobotMap robot) {
        this.brake1=robot.brake1;
        this.brake2=robot.brake2;
    }
    public void update()
    {
        if(currentStatus!=previousStatus){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    this.brake1.setPosition(init_position);
                    this.brake2.setPosition(init_position);
                    break;
                }
                case ON:
                {
                    this.brake1.setPosition(on_position);
                    this.brake2.setPosition(on_position);
                    break;
                }

            }
        }
    }
}
