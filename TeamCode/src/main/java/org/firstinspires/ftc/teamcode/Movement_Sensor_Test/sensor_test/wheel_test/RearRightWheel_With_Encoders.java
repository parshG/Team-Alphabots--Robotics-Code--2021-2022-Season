package org.firstinspires.ftc.teamcode.Movement_Sensor_Test.sensor_test.wheel_test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="RearRightWheel + Encoders", group="Training")
    public class RearRightWheel_With_Encoders extends OpMode {



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
       rearRightWheel();

    }
    @Override
    public void stop() {

    }

    public void rearRightWheel() {
        backRightWheel.setTargetPosition(1000);
        backRightWheel.setPower(0.5);
    }


}