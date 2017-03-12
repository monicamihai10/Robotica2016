package org.firstinspires.ftc.robotcontroller.external.samples;

/*
Modern Robotics Range Sensors Example
Created 10/31/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.35
Reuse permitted with credit where credit is due

Configuration:
I2CDevice "range28" (MRI Range Sensor with default I2C address 0x28
I2CDevice "range2a" (MRI Color Sensor with I2C address 0x2a

ModernRoboticsI2cGyro is not being used because it does not support .setI2CAddr().

To change range sensor I2C Addresses, go to http://modernroboticsedu.com/mod/lesson/view.php?id=96
Support is available by emailing support@modernroboticsinc.com.
*/


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.I2cAddr.create8bit;


public class RangeSensorWrapper {

    public I2cDevice rangeSensor;

    public RangeSensorWrapper(final String sensorName, final HardwareMap hardwareMap, final Integer sensorAddress, final Telemetry telemetry){

        this.rangeSensor = hardwareMap.i2cDevice.get(sensorName);
        if(sensorAddress != null) {
            //rangeSensor.setI2cAddress(create8bit(sensorAddress));
        }

        //this.telemetry = telemetry;
    }



}