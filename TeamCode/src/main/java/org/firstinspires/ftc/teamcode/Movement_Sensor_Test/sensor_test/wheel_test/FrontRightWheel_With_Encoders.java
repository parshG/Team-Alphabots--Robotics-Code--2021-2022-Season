package org.firstinspires.ftc.teamcode.Movement_Sensor_Test.sensor_test.wheel_test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="FrontRightWheel + Encoders", group="Training")
    public class FrontRightWheel_With_Encoders extends OpMode {



    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    double drivePower = 0.5;
     //1 rotation = 360


    @Override
    public void init() {
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    @Override
    public void start() {



    }

    @Override
    public void loop() {
       rightWheel();

    }
    @Override
    public void stop() {

    }

    public void rightWheel() {
        rightWheel.setTargetPosition(1000);
        rightWheel.setPower(0.5);
    }


}