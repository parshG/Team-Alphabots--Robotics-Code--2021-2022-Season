
package org.firstinspires.ftc.teamcode.Autonomous_WORKING_ON;

//parsh is bad
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//adb connect 192.168.43.1:5555
@Disabled
@Autonomous(name="AutoMinus_Blue1_Test3", group="Training")
public class AutoMinus_Blue1_Test3 extends LinearOpMode {

        DcMotor leftWheel;
        DcMotor rightWheel;
        DcMotor backLeftWheel;
        DcMotor backRightWheel;
        DcMotor armMotor;
        DcMotor carouselMotor;
        CRServo intakeServo;
        RevColorSensorV3 colorSensor;
        BNO055IMU imu;
        private Orientation lastAngles = new Orientation();
        private double currAngle = 0.0;
        boolean isStopRequested = false;
        double drivePower = 0.5;
        //1 rotation = 360

        private ElapsedTime runtime = new ElapsedTime();

        @Override
        public void runOpMode() {
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
            leftWheel.setDirection(DcMotor.Direction.REVERSE);
            backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
            setZeroPowerBehaiv();
            setAllMotorPowers(0);


            waitForStart();
            while(opModeIsActive()) {
//                turnRight(25);
//                shippingHubLevel(140, 0.3);
//                sleep(500);
//                encoderMovement(60, 1, 0.5);
//                sleep(500);
//                intakeServo.setPower(1);
//                sleep(1000);
//                intakeServo.setPower(0);
//                sleep(125);
//                intakeServo.setPower(1);
//                sleep(1000);
//                intakeServo.setPower(0);
//                shippingHubLevel(10, 0.04);
//                sleep(1000);
//                encoderMovement(30, 2, 0.5);
//                sleep(500);
//                telemetry.addLine("Before");
//                telemetry.update();
//                turnRight(30);
//                telemetry.addLine("After");
//                telemetry.update();
//                sleep(500);
//                encoderMovement(135, 2, 0.5);
//                sleep(500);
//                turnRight(90);
//                sleep(500);
//                carouselFunc();
//                sleep(500);
//                encoderMovement(69, 1, 0.3);
//                sleep(500);
//
//                break;
                encoderMovement(100, 4, 0.5);
                encoderMovement(100, 4, 0.5);
                break;
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
        public void carouselFunc(){
            carouselMotor.setPower(0.5);
            sleep(2000);
            carouselMotor.setPower(0);
        }
        public void resetAngle(){ //resetting the angles (after we finish turn)
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

        public void turnLeft(double degrees){

            resetAngle();
            double error = degrees;

            while(opModeIsActive() && Math.abs(error) > 2){
                double motorPower = ( error < 0 ? -0.3 : 0.3);
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

    public void forward(int distance, double power){


        final double ENCODER_TPR = 537.6;
        final double GEAR_RATIO = 1;
        final double WHEEL_DIAMETER = 9.6;
        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        final double ROTATIONS = distance / CIRCUMFERENCE;
        double ticks = ENCODER_TPR * ROTATIONS * GEAR_RATIO;
        double error = distance;

        while(opModeIsActive() && Math.abs(error) > 2){
            double motorPower = ( error < 0 ? -ticks : ticks);
            setMotorPowers(motorPower, motorPower, motorPower, motorPower);

            telemetry.addData("error", error);
            telemetry.addData("angle", currAngle);
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", currAngle);
            telemetry.addData("3 correction", error);
            telemetry.update();


        }
        setAllMotorPowers(0);
    }
    public void turnRight(double degrees){

        resetAngle();
        telemetry.addLine("resetting angle");
        telemetry.update();
        double error = degrees;

        while(opModeIsActive() && Math.abs(error) > 2){
            double motorPower = ( error < 0 ? -0.3 : 0.3);
            setMotorPowers(-motorPower, motorPower, -motorPower, motorPower);
            telemetry.addLine("Setting motor powers");
            telemetry.update();
            error = degrees - getAngle();

//            telemetry.addData("error", error);
//            telemetry.addData("angle", currAngle);
//            telemetry.addData("1 imu heading", lastAngles.firstAngle);
//            telemetry.addData("2 global heading", currAngle);
//            telemetry.addData("3 correction", error);
//            telemetry.update();


        }
        setAllMotorPowers(0);
        telemetry.addLine("Setting all motor powers to 0");
        telemetry.update();
    }
        public void turnTo(double degrees) {
            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double error = degrees - orientation.firstAngle;

            if(error > 180) {
                error -= 360;
            } else if(error < -180) {
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
    public void outakeFunc(){
        intakeServo.setPower(-1);
        sleep(3000);
    }


    public void setAllMotorPowers(double power){
        leftWheel.setPower(power);
        rightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }
    public void setMotorPowers(double lw, double rw, double bl, double br){
        leftWheel.setPower(lw);
        telemetry.addLine("Setting left wheel");
        telemetry.update();
        rightWheel.setPower(rw);
        telemetry.addLine("Setting right wheel");
        telemetry.update();
        backLeftWheel.setPower(bl);
        telemetry.addLine("Setting back left wheel");
        telemetry.update();
        backRightWheel.setPower(br);
        telemetry.addLine("Setting back right wheel");
        telemetry.update();
        }
    public void setTargetPositionCounts(double fl_count,double fr_count,double bl_count,double br_count){
        leftWheel.setTargetPosition((int) fl_count);
        rightWheel.setTargetPosition((int) fr_count);
        backLeftWheel.setTargetPosition((int) bl_count);
        backRightWheel.setTargetPosition((int) br_count);
    }
    public void resetEncoders(){
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setRTP(){
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setRUE(){
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setZeroPowerBehaiv(){
        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    }