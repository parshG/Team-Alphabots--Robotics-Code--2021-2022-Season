package org.firstinspires.ftc.teamcode.Autonomous_WORKING_ON;
/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.Movement_Sensor_Test.sensor_test.Sensors_test.SkystoneDeterminationExample;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
@Disabled
@Autonomous(name="AutonomousBlue1_Test", group="Training")
public class AutonomousBlue1_Test extends LinearOpMode
{
    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor armMotor;
    DcMotor carouselMotor;
    CRServo intakeServo;
    RevColorSensorV3 colorSensor;
    BNO055IMU imu;
    boolean isStopRequested = false;
    double drivePower = 0.5;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    //1 rotation = 360
    private final ElapsedTime runtime = new ElapsedTime();
    OpenCvWebcam webcam;
    //1 rotation = 360
    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis = SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // default

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");

        armMotor = hardwareMap.get(DcMotor.class, "expansion_motor");
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel_arm");
        intakeServo = hardwareMap.crservo.get("expansion_servo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        //initializing the IMU and setting the units needed
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationExample.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        waitForStart();
        switch (snapshotAnalysis)
        {
            case LEFT:
            {
                while(opModeIsActive()) {
                    shippingHubLevel(65, 1);
                    sleep(250);
                    encoderMovement(10, 1, 0.2); // Drive forward 10 cm
                    turnRight(30);
                    sleep(250);
                    encoderMovement(60, 1, 0.2);
                    sleep(250);
                    intakeServo.setPower(2);
                    sleep(3000);
                    intakeServo.setPower(0);
                    sleep(1000);
                    encoderMovement(20, 2, 0.2);
                    shippingHubLevel(65, 0.2);
                    sleep(250);
                    turnRight(-30);
                    encoderMovement(100, 4, 0.2);
                    encoderMovement(20, 2, 0.2);
                    carouselFunc();
                    sleep(250);
                    shippingHubLevel(65, 0.2);
                    sleep(250);
                    encoderMovement(45, 1, 0.2);
                    break;
                }


            }
            case RIGHT:
            {
                while(opModeIsActive()) {
                    shippingHubLevel(155, 1);
                    sleep(250);
                    encoderMovement(10, 1, 0.2); // Drive forward 10 cm
                    turnRight(30);
                    sleep(250);
                    encoderMovement(60, 1, 0.2);
                    sleep(250);
                    intakeServo.setPower(2);
                    sleep(3000);
                    intakeServo.setPower(0);
                    sleep(1000);
                    encoderMovement(20, 2, 0.2);
                    shippingHubLevel(65, 0.2);
                    sleep(250);
                    turnRight(-30);
                    encoderMovement(100, 4, 0.2);
                    encoderMovement(20, 2, 0.2);
                    carouselFunc();
                    sleep(250);
                    shippingHubLevel(65, 0.2);
                    sleep(250);
                    encoderMovement(45, 1, 0.2);
                    break;
                }

            }

            case CENTER:
            {
                while(opModeIsActive()) {
                    shippingHubLevel(115, 1);
                    sleep(250);
                    encoderMovement(10, 1, 0.2); // Drive forward 10 cm
                    turnRight(30);
                    sleep(250);
                    encoderMovement(60, 1, 0.2);
                    sleep(250);
                    intakeServo.setPower(2);
                    sleep(3000);
                    intakeServo.setPower(0);
                    sleep(1000);
                    encoderMovement(20, 2, 0.2);
                    shippingHubLevel(65, 0.2);
                    sleep(250);
                    turnRight(-30);
                    encoderMovement(100, 4, 0.2);
                    encoderMovement(20, 2, 0.2);
                    carouselFunc();
                    sleep(250);
                    shippingHubLevel(65, 0.2);
                    sleep(250);
                    encoderMovement(45, 1, 0.2);
                    break;
                }

            }
        }




        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive())
        {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
    public void encoderMovement(double distance, int direction, double power) {

        resetEncoders();
        setRUE();

        final double ENCODER_TPR = 537.6;
        final double GEAR_RATIO = 1;
        final double WHEEL_DIAMETER = 9.6;
        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        final double ROTATIONS = distance / CIRCUMFERENCE;
        double ticks = ENCODER_TPR * ROTATIONS * GEAR_RATIO;

        switch (direction) {
            case 1: // robot will move forward
                setTargetPositionCounts(ticks, ticks, ticks, ticks);
                setAllMotorPowers(power);
                break;
            case 2: // robot will move backward
                setTargetPositionCounts(-ticks, -ticks, -ticks, -ticks);
                setAllMotorPowers(power);

                break;
            case 3: // robot will strafe left
                setTargetPositionCounts(-ticks, ticks, ticks, -ticks);
                setAllMotorPowers(power);
                break;
            case 4: // robot will strafe right
                setTargetPositionCounts(ticks, -ticks, -ticks, ticks);
                setAllMotorPowers(power);
                break;
            case 5: // robot will rotate left
                setTargetPositionCounts(-ticks, ticks, -ticks, ticks);
                setAllMotorPowers(power);
                break;
            case 6: // robot will rotate right
                setTargetPositionCounts(ticks, -ticks, ticks, -ticks);
                setAllMotorPowers(power);
                break;
        }

        setRTP();

        while (leftWheel.isBusy() && rightWheel.isBusy() && backLeftWheel.isBusy() && backRightWheel.isBusy()) {

        }
        resetEncoders();
    }

    public void carouselFunc() {
        carouselMotor.setPower(0.5);
        sleep(2000);
        carouselMotor.setPower(0);
    }

    public void resetAngle() { //resetting the angles (after we finish turn)
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double changeInAngle = orientation.firstAngle - lastAngles.firstAngle; //change in angle from previous angle to current angle

        if (changeInAngle > 180) {
            changeInAngle -= 360;
        } else if (changeInAngle <= -180) {
            changeInAngle += 360;
        }
        //these if statements accommodate for the IMU only going until 180 degrees.

        currAngle += changeInAngle;
        lastAngles = orientation;

        telemetry.addData("gyro", orientation.firstAngle);
        telemetry.update();
        return currAngle;
    }

    public void turnLeft(double degrees) {
        setRWE();
        resetAngle();
        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            setMotorPowers(motorPower, -motorPower, motorPower, -motorPower);
            error = degrees - getAngle();

            telemetry.addData("error", error);
            telemetry.addData("angle", currAngle);
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", currAngle);
            telemetry.addData("3 correction", error);
            telemetry.update();


        }
        setAllMotorPowers(0);
    }

    public void turnRight(double degrees) {
        setRWE();
        resetAngle();
        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            setMotorPowers(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();

            telemetry.addData("error", error);
            telemetry.addData("angle", currAngle);
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", currAngle);
            telemetry.addData("3 correction", error);
            telemetry.update();


        }
        setAllMotorPowers(0);
    }

    public void forward(double degrees, double fl, double fr, double bl, double br) {

        resetAngle();
        double error = degrees;
        runtime.startTime();

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            setMotorPowers(fl, fr, bl, br);
            error = degrees - getAngle();

            telemetry.addData("error", error);
            telemetry.addData("angle", currAngle);
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", currAngle);
            telemetry.addData("3 correction", error);
            telemetry.update();


        }
        setAllMotorPowers(0);
    }

    public void turnTo(double degrees) {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turnLeft(error);

    }

    public void shippingHubLevel(int rotation, double pwr) {
        resetEncoders();
        armMotor.setTargetPosition(rotation);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(pwr);
    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void outakeFunc() {
        intakeServo.setPower(-1);
        sleep(3000);
    }


    public void setAllMotorPowers(double power) {
        leftWheel.setPower(power);
        rightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }

    public void setMotorPowers(double lw, double rw, double bl, double br) {
        leftWheel.setPower(lw);
        rightWheel.setPower(rw);
        backLeftWheel.setPower(bl);
        backRightWheel.setPower(br);
    }

    public void setTargetPositionCounts(double fl_count, double fr_count, double bl_count, double br_count) {
        leftWheel.setTargetPosition((int) fl_count);
        rightWheel.setTargetPosition((int) fr_count);
        backLeftWheel.setTargetPosition((int) bl_count);
        backRightWheel.setTargetPosition((int) br_count);
    }

    public void resetEncoders() {
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setRTP() {
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setRUE() {
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setZeroPowerBehaiv() {
        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setRWE() {
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

