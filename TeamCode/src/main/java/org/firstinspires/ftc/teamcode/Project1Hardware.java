package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;

public class Project1Hardware {
    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotorEx slider, arm, lRigging, rRigging;
    ServoImplEx leftClaw, rightClaw, clawP, drone;
    ServoImplEx lRiggingUp, rRiggingUp;
    //DistanceSensor leftDis, rightDis;
    HardwareMap hwmap;
    IMU imu1;

    public Project1Hardware(HardwareMap hardwareMap) {
        hwmap = hardwareMap;
    }

    public Project1Hardware() {
    }

    public void init(HardwareMap hardwareMap) {
        hwmap = hardwareMap;
        imu1 = hardwareMap.get(IMU.class, "imu");

        frontLeft = hardwareMap.get(DcMotor.class, "motorFL");
        frontRight = hardwareMap.get(DcMotor.class, "motorFR");
        backLeft = hardwareMap.get(DcMotor.class, "motorBL");
        backRight = hardwareMap.get(DcMotor.class, "motorBR");
        slider = hardwareMap.get(DcMotorEx.class, "slider");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        lRigging = hardwareMap.get(DcMotorEx.class,"lRigging");
        rRigging = hardwareMap.get(DcMotorEx.class,"rRigging");

        leftClaw = hardwareMap.get(ServoImplEx.class, "lClaw");
        rightClaw = hardwareMap.get(ServoImplEx.class, "rClaw");
        clawP = hardwareMap.get(ServoImplEx.class, "clawP");
        drone = hardwareMap.get(ServoImplEx.class,"drone");
        lRiggingUp = hardwareMap.get(ServoImplEx.class,"lRiggingUp");
        rRiggingUp = hardwareMap.get(ServoImplEx.class,"rRiggingUp");

        /*leftDis = hardwareMap.get(DistanceSensor.class, "lDis");
        rightDis = hardwareMap.get(DistanceSensor.class, "rDis");*/

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lRigging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rRigging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lRigging.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rRigging.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        slider.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        lRigging.setDirection(DcMotorEx.Direction.FORWARD);
        rRigging.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lRigging.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rRigging.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // TODO: Change direction
        leftClaw.setDirection(ServoImplEx.Direction.FORWARD);
        rightClaw.setDirection(ServoImplEx.Direction.REVERSE);
        clawP.setDirection(ServoImplEx.Direction.REVERSE);
        drone.setDirection(Servo.Direction.FORWARD);


        //clawP.setPosition(0.5);

        imu1.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
    }

    public void setSlider(int pos) {

        if(pos>900){
            pos=900;
        }
        slider.setTargetPosition(pos + (int) (arm.getCurrentPosition() / 19));
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1);
    }
    public void setScoringArm(double angle) {
        arm.setTargetPosition(angleToEncoderValueArm(angle));
    }
    public void setClaw() {
        leftClaw.setPosition(0.05);
        rightClaw.setPosition(0);
    }
    public void setDrone() {
        drone.setPosition(0);
    }
    public void setRiggingServo() {
        lRiggingUp.setPosition(1);
        rRiggingUp.setPosition(1);
    }
    //Movements
    public void extendRiggingServo() {
        lRiggingUp.setPosition(0.5);
        rRiggingUp.setPosition(0.5);
    }
    public void retractRiggingMotor() {
        lRigging.setPower(-1);
        rRigging.setPower(-1);
    }
    public void extendRiggingMotor() {
        lRigging.setPower(1);
        rRigging.setPower(1);
    }
    public void droneLaunch() {
        drone.setPosition(0.8);
    }
    public void bothClawOpen() {
        leftClaw.setPosition(0.05);
        rightClaw.setPosition(0);
    }

    public void leftClawOpen() {
        leftClaw.setPosition(0);
    }

    public void rightClawOpen() {
        rightClaw.setPosition(0.05);
    }

    public void leftClawClose() {
        leftClaw.setPosition(0.55);
        leftClaw.setPosition(0.6);
    }

    public void rightClawClose() {
        rightClaw.setPosition(0.55);
        rightClaw.setPosition(0.6);
    }

    public void bothClawClose() {
        rightClawClose();
        leftClawClose();
    }

    //Extensions
    public void extendSlider() {
        slider.setPower(1);
        slider.setTargetPosition(1000);
        slider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    //Retractions
    public void retractSlider() {
        slider.setPower(1);
        slider.setTargetPosition(0);
        slider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setClawPAngle(double angle) {
        clawP.setPosition(angle / 174);
    }

    public int angleToEncoderValueArm(double angle) {
        double CPR = 3895.9;
        double revolutions = (angle + 12) / 360;
        double tmp = revolutions * CPR;
        return (int) tmp;
    }

    public void setArm(double angle) {
        int tmp;
        arm.setVelocity(1300);

        tmp=angleToEncoderValueArm(angle);
        if(tmp>1900){
            tmp=1900;
        }
        arm.setTargetPosition(angleToEncoderValueArm(angle));
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getArmAngle() {
        double CPR = 3895.9;
        int position = arm.getCurrentPosition();
        double revolutions = position / CPR;
        double angle = revolutions * 360 - 12;
        return angle;
    }

    public int lengthToEncoderValueSlider(double length) {
        double CPR = 145.1 * 1.4;
        double revolutions = length / 35.65 / Math.PI;
        double tmp = revolutions * CPR;
        return (int) tmp;
    }

    public void setSliderLength(double length) {
        // if(length < 10) length = 0;
        //if (length > 1000) length = 1000;
        slider.setTargetPosition(lengthToEncoderValueSlider(length));
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1);
    }

    public double getSliderLength() {
        double CPR = 145.1 * 1.4;
        int position = slider.getCurrentPosition();
        double revolutions = position / CPR;
        double length = revolutions * 35.65 * Math.PI;
        return length;
    }
    public void clawRIntake(){
        setClawPAngle(90 - getArmAngle() - 18);
    }

    /*public double getDis() {
        double avgDis = ((leftDis.getDistance(DistanceUnit.CM) + rightDis.getDistance(DistanceUnit.CM))/2);
        return avgDis;
                    */
}