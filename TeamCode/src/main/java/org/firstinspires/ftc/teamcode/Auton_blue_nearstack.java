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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Disabled
@Autonomous(name = "auton_blue_nearstack")
public class Auton_blue_nearstack extends LinearOpMode {
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
        FINISH,
    }
    camera_stage cameraStage = camera_stage.UNKNOWN;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Auton_blue_NSHardware robot = new Auton_blue_NSHardware(hardwareMap);
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-40, 64, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        ElapsedTime timer3 = new ElapsedTime();
        ElapsedTime timer4 = new ElapsedTime();
        double cur = timer.milliseconds();
        double cur2 = timer2.milliseconds();
        double last=0;
        double distance=0;

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(-32.00, 64.00, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-40, 47.77), Math.toRadians(270.00))
                .lineToSplineHeading(new Pose2d(-36.60, 31.77,Math.toRadians(180)))
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

                .splineTo(new Vector2d(-30.34, 11.40), Math.toRadians(0.00),SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToConstantHeading(new Vector2d(38.08, 14.31))
                .splineTo(new Vector2d(54.17, 44), Math.toRadians(0.00),SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{
                    scoringStage = scoring_stage.SET;
                })
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(-32.00, 64.00, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(-38.54, 18),SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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

                .splineTo(new Vector2d(-28.34, 11.40), Math.toRadians(0.00),SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToConstantHeading(new Vector2d(38.08, 14.31))
                .splineTo(new Vector2d(55.00, 33.43), Math.toRadians(0.00),SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{
                    scoringStage = scoring_stage.SET;
                })
                .build();


        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(-40, 64.00, Math.toRadians(270.00)))
                .splineToConstantHeading(new Vector2d(-47.85, 19.15), Math.toRadians(215.00))
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

                .splineTo(new Vector2d(-36.34, 11.40), Math.toRadians(0.00),SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToConstantHeading(new Vector2d(38.08, 14.31))
                .splineTo(new Vector2d(55,32), Math.toRadians(0.00),SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
            cur2 =timer.milliseconds();

            /*
            if (gamepad1.dpad_left){
                cameraStage = camera_stage.LEFT;
            } else if (gamepad1.dpad_up){
                cameraStage =camera_stage.MIDDLE;
            } else if (gamepad1.dpad_right){
                cameraStage = camera_stage.RIGHT;
            }
            if (gamepad1.square){
                scoringStage = scoring_stage.READY;
            }else if (gamepad1.triangle){
                scoringStage = scoring_stage.CLAMP;
            }else if (gamepad1.circle){
                scoringStage = scoring_stage.SLIDERSUP;
            }else if (gamepad1.cross){
                scoringStage = scoring_stage.RELEASE;
            }
             */
            switch (cameraStage){
                case LEFT:
                    drive.followTrajectorySequence(left);
                    cameraStage = camera_stage.FINISH;
                    break;
                case RIGHT:
                    drive.followTrajectorySequence(right);
                    cameraStage = camera_stage.FINISH;
                    break;
                case MIDDLE:
                    drive.followTrajectorySequence(middle);
                    cameraStage = camera_stage.FINISH;
                    break;
                case FINISH:
                    break;
            }
            switch (scoringStage){
                case READY:
                    if (cur>800){
                        robot.bothClawOpen();
                    }
                    if (cur>1000){
                        robot.setArm(0); // TODO: set arm pos to scoring pos
                        robot.setSlider(5); //TODO: set slider pos to scoring pos
                        robot.clawP.setPosition(0.5); // TODO: set claw orientation to scoring pos
                    }
                    //if (cur>700 && (robot.bbBack.getState()|| robot.bbFront.getState())){
                    if (cur>700 && gamepad1.right_bumper){
                        cur=0;
                        timer.reset();
                        scoringStage = scoring_stage.CLAMP;
                    }
                    break;
                case CLAMP:
                    robot.retractSlider();
                    robot.setArm(0); // TODO: set arm pos to intake pos
                    if (cur>400){
                        robot.bothClawClose();
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
                        robot.setArm(0); // TODO:  set arm pos to scoring pos
                        robot.setSlider(robot.heightplaced); // TODO: set heightplaced value or slider pos
                    }
                    if ((robot.heightplaced -robot.slider.getCurrentPosition()<10)){
                        robot.clawP.setPosition(0); // TODO: set claw orientation to ??? pos
                        cur=0;
                        timer.reset();
                        scoringStage = scoring_stage.RELEASE;
                    }
                    break;
                case RELEASE:
                    if (cur>200){
                        robot.rightClawOpen();
                    }
                    if (cur>600){
                        robot.setArm(0); // TODO: set arm pos to scoring pos
                        robot.setSlider(0); // TODO: set slider pos to scoring pos
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
