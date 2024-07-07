package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Disabled
@Autonomous(name = "auton_blue_nearboard")
public class Auton_blue_nearboard extends LinearOpMode {
    public enum scoring_stage{
        READY,
        SET,
        CLAMP,
        SLIDERSUP,
        RELEASE,
    }
    scoring_stage scoringStage = scoring_stage.CLAMP;
    public enum camera_stage{
        UNKNOWN,
        LEFT,
        RIGHT,
        MIDDLE,
        SCORING,
        FINISH,
        END
    }
    camera_stage cameraStage = camera_stage.UNKNOWN;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Auton_blue_NBHardware robot = new Auton_blue_NBHardware(hardwareMap);
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(16, 64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        ElapsedTime timer3 = new ElapsedTime();
        ElapsedTime timer4 = new ElapsedTime();
        double cur = timer.milliseconds();
        double cur2 = timer2.milliseconds();
        double last=0;
        int pixelplaced=0;
        boolean completed=false;

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(36.34, 52.57), Math.toRadians(-61.95))
                .lineToSplineHeading(new Pose2d(34.77, 32.00, Math.toRadians(0.00)))
                .addTemporalMarker(() -> {
                    robot.leftClawOpen();
                })
                .waitSeconds(1)

                /*
                .addTemporalMarker(() -> {
                    robot.intake.setPower(0);
                    robot.Intakeserv.setPower(0);
                })
                */

                .lineToConstantHeading(new Vector2d(56.93, 43.43),SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{
                    scoringStage = scoring_stage.SET;
                })
                .build();


        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    robot.leftClawOpen();
                })
                .waitSeconds(1)

                /*
                .addTemporalMarker(() -> {
                    robot.intake.setPower(0);
                    robot.Intakeserv.setPower(0);
                })
                */

                .lineToSplineHeading(new Pose2d(55, 34.43, Math.toRadians(0.00)),SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{
                    scoringStage = scoring_stage.SET;
                })
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(10.29, 31.54, Math.toRadians(0.00)))
                .addTemporalMarker(() -> {
                    robot.leftClawOpen();
                })
                .waitSeconds(1)

                /*
                .addTemporalMarker(() -> {
                    robot.intake.setPower(0);
                    robot.Intakeserv.setPower(0);
                })
                */

                .lineToConstantHeading(new Vector2d(54.17, 30),SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{
                    scoringStage = scoring_stage.SET;
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
        scoringStage = scoring_stage.CLAMP;


        while (opModeIsActive()){
            cur = timer.milliseconds();
            cur2 = timer.milliseconds();

            if (pixelplaced == 1){
                completed = true;
            }
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
                case FINISH:
                    robot.retractSlider();
                    break;
            }
            switch (scoringStage){
                case READY:
                    if (completed!=true) {
                        if (cur > 800) {
                            robot.bothClawOpen();
                        }
                        if (cur > 1000) {
                            robot.setArm(0); // TODO: set arm to scoring pos
                            robot.setSlider(5); // TODO: set slider pos to scoring pos
                            robot.clawP.setPosition(0.5); // TODO: set orientation to scoring orientation
                        }
                        if (cur > 700){
                            cur = 0;
                            timer.reset();
                            scoringStage = scoring_stage.CLAMP;
                        }
                    }
                    if (completed){
                        cameraStage=camera_stage.FINISH;
                    }
                    break;
                case CLAMP:
                    robot.retractSlider();
                    robot.setArm(0); // TODO: set arm pos to intake pos
                    if (cur>400){
                        robot.bothClawClose(); // TODO: Might need to change???
                    }
                    if (cur>600){
                        cur=0;
                        timer.reset();
                        scoringStage=scoring_stage.READY;
                    }
                    break;
                case SET:
                    if (cur>1){
                        cur=0;
                        timer.reset();
                        scoringStage = scoring_stage.SLIDERSUP;
                    }
                    break;
                case SLIDERSUP:
                    if (cur>400){
                        robot.setArm(0); // TODO: set arm pos to scoring pos
                        robot.setSlider(robot.heightplaced); // TODO: set heightplaced value or slider pos
                    }
                    if ((robot.heightplaced - robot.slider.getCurrentPosition()<10)){
                        robot.clawP.setPosition(0); // TODO: set claw orientation to ??? pos
                            cur=0;
                            timer.reset();
                            scoringStage = scoring_stage.RELEASE;
                    }
                    break;
                case RELEASE:
                    pixelplaced+=1;
                    if (cur>200){
                        robot.rightClawOpen();
                    }
                    if (cur>600){
                        robot.setArm(0); // TODO: Set swing to scoring pos
                        robot.setSlider(0); // TODO: set slider to scoring pos
                    }
                    if (cur>600){
                        cur=0;
                        timer.reset();
                        scoringStage = scoring_stage.READY;
                    }
                    break;

            }
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.update();
        }
    }
}
