package org.firstinspires.ftc.teamcode.writtenCode.controllers;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.writtenCode.RobotMap;

@Config

@Configurable
public class HoodController {
    public enum HoodStatus{
        INIT,
        RUNTO;
    }
    public HoodStatus currentStatus= HoodStatus.INIT;
    public HoodStatus previousStatus=null;

    public static double hoodInitPosition=1;




    public Servo hood = null;

    public HoodController(RobotMap robot){this.hood=robot.hood;}
    public void update(double runto_target){

        if(currentStatus!=previousStatus || currentStatus== HoodStatus.RUNTO)
        {
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    this.hood.setPosition(hoodInitPosition);
                    break;
                }
                case RUNTO:
                {
                    this.hood.setPosition(runto_target);
                    break;
                }

            }
        }
    }

}

