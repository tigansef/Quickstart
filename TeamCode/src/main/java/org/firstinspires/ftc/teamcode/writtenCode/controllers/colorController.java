package org.firstinspires.ftc.teamcode.writtenCode.controllers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "colorsensor")
public class colorController extends OpMode {

    private ColorSensor sensor1;
    private ColorSensor sensor2;
    private ColorSensor sensor3;
    int r1;
    int g1;
    int b1;
    int r2;
    int g2;
    int b2;
    int r3;
    int g3;
    int b3;
    int a1;
    int a2;
    int a3;
    private DistanceSensor dist1;
    private DistanceSensor dist2;
    private DistanceSensor dist3;
    double d1;
    double d2;
    double d3;
    boolean treibile;

    @Override
    public void init(){

        sensor1=hardwareMap.get(ColorSensor.class, "sensor1");
        sensor2=hardwareMap.get(ColorSensor.class, "sensor2");
        sensor3=hardwareMap.get(ColorSensor.class, "sensor3");
        sensor1.enableLed(true);
        sensor2.enableLed(true);
        sensor3.enableLed(true);
        dist1 = hardwareMap.get(DistanceSensor.class, "sensor1");
        dist2 = hardwareMap.get(DistanceSensor.class, "sensor2");
        dist3 = hardwareMap.get(DistanceSensor.class, "sensor3");
        treibile=false;
    }

    @Override
    public void loop(){
        r1 = sensor1.red();
        g1 = sensor1.green();
        b1 = sensor1.blue();

        r2 = sensor2.red();
        g2 = sensor2.green();
        b2 = sensor2.blue();

        r3 = sensor3.red();
        g3 = sensor3.green();
        b3 = sensor3.blue();

        a1 = sensor1.alpha();
        a2 = sensor2.alpha();
        a3 = sensor3.alpha();
        d1 = dist1.getDistance(DistanceUnit.CM);
        d2 = dist2.getDistance(DistanceUnit.CM);
        d3 = dist3.getDistance(DistanceUnit.CM);
        if(d1<8 && d2<5 && d3<4.5) treibile=true;
        else treibile=false;

        telemetry.addData("Sensor1 R G B Alpha Dist", r1 + " " + g1 + " " + b1 + " " + a1 + " " + d1);
        telemetry.addData("Sensor2 R G B Alpha Dist", r2 + " " + g2 + " " + b2 + " " + a2 + " " + d2);
        telemetry.addData("Sensor3 R G B Alpha Dist", r3 + " " + g3 + " " + b3 + " " + a3 + " " + d3);
        telemetry.addData("trei bile", treibile);
        telemetry.update();
    }
}