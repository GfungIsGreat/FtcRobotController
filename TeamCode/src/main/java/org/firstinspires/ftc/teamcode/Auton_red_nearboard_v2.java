package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "auton_red_nearboard_v2")
public class Auton_red_nearboard_v2 extends LinearOpMode {
    public enum scoring_stage{
        INIT,
        GROUND,
        GROUND_EXTEND,
        GROUND_GRIP,
        EXTEND_GRIP,
        READY_SCORE,
        SCORING,
        RETURN_TO_GROUND,
    }
    scoring_stage scoringStage = scoring_stage.READY_SCORE;
    public enum camera_stage{
        UNKNOWN,
        LEFT,
        RIGHT,
        MIDDLE,
        SCORING,
        PARKING,
        FINISH,
    }
    camera_stage cameraStage = camera_stage.UNKNOWN;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Auton_red_NBHardware robot = new Auton_red_NBHardware(hardwareMap);
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(16, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        ElapsedTime timer3 = new ElapsedTime();
        ElapsedTime timer4 = new ElapsedTime();
        double cur = timer.milliseconds();
        double cur2 = timer2.milliseconds();
        double last=0;

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(10.29, -31.54, Math.toRadians(0.00)))
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(36.7, -31.00, Math.toRadians(0.00)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{
                    scoringStage = scoring_stage.READY_SCORE;
                })
                .build();



        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(17.19, -38.15, Math.toRadians(270)))
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(36.7, -31.00, Math.toRadians(0.00)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{
                    scoringStage = scoring_stage.READY_SCORE;
                })
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(35.25, -52.57), Math.toRadians(43.85))
                .lineToSplineHeading(new Pose2d(36.7, -31.00, Math.toRadians(0.00)),SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.waitSeconds(1)
                // .lineToConstantHeading(new Vector2d(55, -47)) Parking
                .addTemporalMarker(()->{
                    scoringStage = scoring_stage.READY_SCORE;
                })
                .build();

        TrajectorySequence lparking = drive.trajectorySequenceBuilder(left.end())
                .lineToConstantHeading(new Vector2d(50,-10.85))
                .addTemporalMarker(()->{
                    cameraStage = camera_stage.FINISH;
                })
                .build();

        TrajectorySequence mparking = drive.trajectorySequenceBuilder(middle.end())
                .lineToConstantHeading(new Vector2d(50,-10.85))
                .addTemporalMarker(()->{
                    cameraStage = camera_stage.FINISH;
                })
                .build();

        TrajectorySequence rparking = drive.trajectorySequenceBuilder(right.end())
                .lineToConstantHeading(new Vector2d(50,-10.85))
                .addTemporalMarker(()->{
                    cameraStage = camera_stage.FINISH;
                })
                .build();

        while (!opModeIsActive()){
            telemetry.addLine("robot.initialized");
            if (robot.teamprop_position==0){
                //left
                //cameraStage = camera_stage.LEFT;
                telemetry.addLine("left");
                cameraStage=camera_stage.LEFT;
            } else if (robot.teamprop_position==1){
                //center
                //cameraStage = camera_stage.MIDDLE;
                telemetry.addLine("middle");
                cameraStage=camera_stage.MIDDLE;
            }else if (robot.teamprop_position==2){
                //right
                //cameraStage = camera_stage.RIGHT;
                cameraStage=camera_stage.RIGHT;
                telemetry.addLine("right");
            }
            telemetry.update();
        }

        waitForStart();
        timer.reset();


        //robot set-up
        robot.bothClawClose();
        robot.setArm(-12);
        robot.setClawPAngle(170);
        robot.setSlider(0);
        robot.setDrone();
        //robot.setRiggingServo();

        /*lDis = robot.leftDis.getDistance(DistanceUnit.CM);
        rDis = robot.rightDis.getDistance(DistanceUnit.CM);*/

        robot.imu1.resetYaw();
        robot.slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cur =0;
        timer.reset();
        scoringStage = scoring_stage.READY_SCORE;


        while (opModeIsActive()){
            cur = timer.milliseconds();
            cur2 =timer.milliseconds();

            switch (cameraStage){
                case LEFT:
                    drive.followTrajectorySequence(left);
                    cameraStage = camera_stage.SCORING;
                    break;
                case RIGHT:
                    drive.followTrajectorySequence(right);
                    cameraStage = camera_stage.SCORING;
                    break;
                case MIDDLE:
                    drive.followTrajectorySequence(middle);
                    cameraStage = camera_stage.SCORING;
                    break;
                case SCORING:
                    break;
                case PARKING:
                    if (robot.teamprop_position==2){
                        drive.followTrajectorySequence(rparking);
                        cameraStage=camera_stage.FINISH;
                    }
                    if (robot.teamprop_position==0){
                        drive.followTrajectorySequence(lparking);
                        cameraStage=camera_stage.FINISH;
                    }
                    if (robot.teamprop_position==1){
                        drive.followTrajectorySequence(mparking);
                        cameraStage=camera_stage.FINISH;
                    }
                    break;
                case FINISH:
                    if (cur > 800) {
                        robot.leftClawOpen();
                    }
                    if (cur > 1000) {
                        robot.setArm(160);
                        robot.setSlider(900);
                        robot.setClawPAngle(180 - (robot.getArmAngle() - 30));
                        drive.setMotorPowers(0,0,0,0);
                    }
                    break;
            }
            switch (scoringStage){
                case GROUND_GRIP:
                    cameraStage= camera_stage.FINISH;
                    break;
                case INIT:
                    robot.retractSlider();
                    robot.setArm(0);
                    robot.clawRIntake();

                    if (cur>200){
                        robot.bothClawOpen(); // TODO: Might need to change???
                    }
                    if (cur>400){
                        cur=0;
                        timer.reset();
                        scoringStage= scoring_stage.GROUND_GRIP;
                    }
                    break;
                case SCORING:
                    if (cur>400){
                        robot.setArm(160);
                        robot.setSlider(robot.heightplaced); // TODO: set heightplaced value or slider pos (900 is scoring pos)
                    }
                    if ((robot.heightplaced - robot.slider.getCurrentPosition()<10)){
                        robot.setClawPAngle(180 - robot.getArmAngle() +20);
                        robot.rightClawOpen();
                        cur=0;
                        timer.reset();
                        scoringStage = scoring_stage.GROUND;
                    }
                    break;
                case READY_SCORE:
                    if (cur>200){
                        cur=0;
                        timer.reset();
                        scoringStage = scoring_stage.SCORING;
                    }
                case GROUND:
                    robot.setSlider(800); // TODO: tune slider pos
                    robot.setArm(0);
                    if (cur>200){
                        robot.rightClawOpen();
                    }
                    if (cur>600){
                        cur=0;
                        timer.reset();
                        scoringStage = scoring_stage.INIT;
                    }
                    break;

            }
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("camera",cameraStage);
            telemetry.update();
        }
    }
}