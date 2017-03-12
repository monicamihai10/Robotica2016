package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ColorSensorWrapper {

    private ColorSensor colorSensor;
    private Telemetry telemetry;
    private boolean white;

    public ColorSensorWrapper(final String sensorName, final HardwareMap hardwareMap, final Integer sensorAddress, final Telemetry telemetry){
        this.colorSensor = hardwareMap.colorSensor.get(sensorName);
        if(sensorAddress != null) {
            colorSensor.setI2cAddress(I2cAddr.create8bit(sensorAddress));
        }
        markSensor(false);
        this.telemetry = telemetry;
    }

    public void enableLed(){
        markSensor(true);
    }

    private void markSensor(final boolean val){
        try {
            colorSensor.enableLed(val);
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void printValues(){
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red-A  ", colorSensor.red());
        telemetry.addData("Green-A  ", colorSensor.green());
        telemetry.addData("Blue-A  ", colorSensor.blue());
        telemetry.addData("Address ", colorSensor.getI2cAddress().get7Bit());
    }

    public boolean isWhite() {

            if((colorSensor.alpha()>15) && (colorSensor.red()>5)&& (colorSensor.green()>5)&& (colorSensor.blue()>5)){
                return true;
            }else{
                return false;
            }}

    public boolean isMov() {

        if((colorSensor.red()!=0)&&(colorSensor.blue()!=0)){
            return true;
        }else{
            return false;
        }}

    public boolean isRed() {

        if((colorSensor.alpha()>15) && (colorSensor.red()>5)&& (colorSensor.green()<5)&& (colorSensor.blue()<5)){
            return true;
        }else{
            return false;
        }}

    public boolean isBlue() {

        if((colorSensor.alpha()>15) && (colorSensor.red()<5)&& (colorSensor.green()<5)&& (colorSensor.blue()>5)){
            return true;
        }else{
            return false;
        }}


//        trebuie sa interpretam valorile returnate de red/green/blue
     //   return white;
    }

