package org.firstinspires.ftc.teamcode.writtenCode.controllers;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.writtenCode.RobotMap;

@Config
@Configurable
public class PtoController {
    public enum PtoStatus{
        ON,
        OFF;
    }
    public PtoStatus currentStatus= PtoStatus.OFF;
    public PtoStatus previousStatus=null;

    public static double ptoOffPosition=0;
    public static double ptoOnPosition=0;

    public Servo pto = null;

    public PtoController(RobotMap robot){this.pto=robot.pto;}
    public void update()
    {
        if(currentStatus!=previousStatus){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case OFF:
                {
                    this.pto.setPosition(ptoOffPosition);
                    break;
                }
                case ON:
                {
                    this.pto.setPosition(ptoOnPosition);
                    break;
                }
            }
        }
    }
}

