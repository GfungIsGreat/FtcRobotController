package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class Pipeline extends LinearOpMode {
    public OpenCvCamera webcam1 = null;
    WebcamName webcamName = null;
    public static int teamprop_position;

    class redPipeline extends OpenCvPipeline {
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

            Rect leftRect = new Rect(1, 160, 100, 100);
            Rect MidRect = new Rect(250, 160, 100, 100);
            Rect rightRect = new Rect(530, 160, 100, 100);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, MidRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = HSV.submat(leftRect);
            midCrop = HSV.submat(MidRect);
            rightCrop = HSV.submat(rightRect);

            //creating coundaries for red
            Scalar lowHSV = new Scalar(0, 80, 70); //lenient lower bound
            Scalar highHSV = new Scalar(10, 255, 255);//lenient higher bound

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

            //telemetry.addData("Leftavg", leftavg.val[0]);
            //telemetry.addData("Midavg", midavg.val[0]);
            //telemetry.addData("Rightavg", rightavg.val[0]);
            return (outPut);
        }
    }

    class bluePipeline extends OpenCvPipeline {
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

            Rect leftRect = new Rect(30, 100, 100, 79);
            Rect MidRect = new Rect(300, 100, 100, 79);
            Rect rightRect = new Rect(539, 100, 100, 79);

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

    @Override
    public void runOpMode() throws InterruptedException {

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam1, 0);

        webcam1.setPipeline(new redPipeline());
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });

        waitForStart();
        while (opModeIsActive()) {

            /*if (teamprop_position == 0) {
                telemetry.addLine("LEFT");
            }

            if (teamprop_position == 1) {
                telemetry.addLine("MIDDDLE");
            }

            if (teamprop_position == 2) {
                telemetry.addLine("RIGHT");
            }*/

            telemetry.addData("teamprop", teamprop_position);
            telemetry.update();
        }
    }
}