package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Auton_blue_NSHardware {

    public DcMotorEx slider, arm, lRigging, rRigging;
    public ServoImplEx leftClaw = null, rightClaw = null, clawP = null, drone = null;
    // public DistanceSensor leftDis = null, rightDis = null;
    ServoImplEx lRiggingUp, rRiggingUp;

    HardwareMap hwmap = null;
    public IMU imu1;
    //DcMotor encoder;

    WebcamName webcamName = null;

    public OpenCvCamera webcam1 = null;

    public static int teamprop_position;
    //DistanceSensor leftDis,rightDis,backDis,fRightDis;

    public Auton_blue_NSHardware(HardwareMap hardwareMap) {
        hwmap = hardwareMap;
    }


    public void init(HardwareMap hardwareMap) {
        hwmap = hardwareMap;
        imu1 = hardwareMap.get(IMU.class, "imu");

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

        /* leftDis = hardwareMap.get(DistanceSensor.class, "lDis");
        rightDis = hardwareMap.get(DistanceSensor.class, "rDis"); */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        slider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lRigging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rRigging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lRigging.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rRigging.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slider.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        lRigging.setDirection(DcMotorEx.Direction.FORWARD);
        rRigging.setDirection(DcMotorSimple.Direction.REVERSE);

        slider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lRigging.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rRigging.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // TODO: Change direction
        leftClaw.setDirection(ServoImplEx.Direction.FORWARD);
        rightClaw.setDirection(ServoImplEx.Direction.REVERSE);
        clawP.setDirection(ServoImplEx.Direction.REVERSE);
        drone.setDirection(Servo.Direction.FORWARD);


        //clawR.setPosition(0.5);

        imu1.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam1.setPipeline(new blueRightPipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            public void onError(int errorCode) {
            }
        });
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

    public int heightplaced = 650;

    //BLUE ONLY - OpenCv team prop recognition code
    public class blueRightPipeline extends OpenCvPipeline {
        Mat HSV = new Mat();
        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;
        double leftavgfin;
        double midavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
        Scalar boundingRect = new Scalar(60.0, 255, 255);

        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            //telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(0, 160, 150, 100);
            Rect MidRect = new Rect(590, 160, 150, 100);
            Rect rightRect = new Rect(1130, 160, 150, 100);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, MidRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = HSV.submat(leftRect);
            midCrop = HSV.submat(MidRect);
            rightCrop = HSV.submat(rightRect);

            //creating coundaries for blue
            Scalar lowHSV = new Scalar(80, 80, 70); //lenient lower bound
            Scalar highHSV = new Scalar(110, 240, 255);

            //appying red filter
            Core.inRange(leftCrop, lowHSV, highHSV, leftCrop);
            Core.inRange(midCrop, lowHSV, highHSV, midCrop);
            Core.inRange(rightCrop, lowHSV, highHSV, rightCrop);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            midavgfin = midavg.val[0];
            rightavgfin = rightavg.val[0];

            if (leftavgfin > midavgfin && leftavgfin > rightavgfin) {
                //telemetry.addLine("Left");
                Imgproc.rectangle(outPut, leftRect, boundingRect, -1);
                teamprop_position = 0;
            } else if (midavgfin > rightavgfin && midavgfin > leftavgfin) {
                //telemetry.addLine("Middle");
                Imgproc.rectangle(outPut, MidRect, boundingRect, -1);
                teamprop_position = 1;
            } else if (rightavgfin > midavgfin && rightavgfin > leftavgfin) {
                //telemetry.addLine("Right");
                Imgproc.rectangle(outPut, rightRect, boundingRect, -1);
                teamprop_position = 2;
            }

            return outPut;
        }
    }
}