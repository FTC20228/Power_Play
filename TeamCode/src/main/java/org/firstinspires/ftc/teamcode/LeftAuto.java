package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Left", group="Autonomous")

public class LeftAuto extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        double open = 0;
        double closed = 1;

        drive.servoR.setDirection(Servo.Direction.REVERSE);
        drive.servoA.setDirection(Servo.Direction.REVERSE);

        drive.servoR.scaleRange(0.24, 0.9);
        drive.servoL.scaleRange(0.24, 0.9);
        drive.servoA.scaleRange(0.23, 0.88);//change to 0.88
        drive.servo0.scaleRange(0.8, 0.97);

        drive.servoL.setPosition(0.5);
        drive.servoR.setPosition(0.5);
        drive.servoA.setPosition(1);
        drive.servo0.setPosition(1);

        //drive.setPoseEstimate(new Pose2d());

        Trajectory toScan = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(30)
                .build();

        Trajectory toPost = drive.trajectoryBuilder(toScan.end())
                .addTemporalMarker(0.1, () -> {
                    armMove(2250); //Life Arm to Height
                })
                .strafeRight(32)
                .build();

        Trajectory PosttoStackP1 = drive.trajectoryBuilder(toPost.end())
                .strafeRight(14)
                .build();

        Trajectory PosttoStackP2 = drive.trajectoryBuilder(PosttoStackP1.end())
                .addTemporalMarker(0.1, () ->{
                    armMove(500); //Life Arm to Height

                })
                .forward(37) //was 34
                .build();

        TrajectorySequence StacktoPost1 = drive.trajectorySequenceBuilder(PosttoStackP2.end())
                .addTemporalMarker(0.1, () -> {
                    armMove(2250); //Life Arm to Height
                })
                .back(37)
                .strafeLeft(10)
                .turn(Math.toRadians(-39))
                .build();

        TrajectorySequence parkRed = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(39))
                .forward(30)
                .build();

        TrajectorySequence parkBlue = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(39))
                .back(30)
                .build();

        TrajectorySequence parkGreen = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-39))
                .build();

        drive.colorR.enableLed(true);

        waitForStart();

        drive.followTrajectory(toScan);
        if (drive.colorR.red()>drive.colorR.green() && drive.colorR.red()>drive.colorR.blue()) {
            telemetry.addLine("Red");
            drive.followTrajectory(toPost);
            Gripper(1,1);
            sleep(500);
            drive.servo0.setPosition(open);
            sleep(300);
            Gripper(0,0);
            sleep(200);
            drive.followTrajectory(PosttoStackP1);
            drive.followTrajectory(PosttoStackP2);
            drive.servo0.setPosition(closed);
            sleep(300);
            Gripper(0,0.5);
            sleep(300);
            drive.followTrajectorySequence(StacktoPost1);
            Gripper(1,1);
            sleep(400);
            drive.servo0.setPosition(open);
            sleep(300);
            Gripper(0,0);
            sleep(300);
            drive.followTrajectorySequence(parkRed);
            armMove(0);
            sleep(500);

        } else if (drive.colorR.green()>drive.colorR.blue() && drive.colorR.green()>drive.colorR.red()) {
            telemetry.addLine("Green");
            drive.followTrajectory(toPost);
            Gripper(1,1);
            sleep(500);
            drive.servo0.setPosition(open);
            sleep(300);
            Gripper(0,0);
            sleep(200);
            drive.followTrajectory(PosttoStackP1);
            drive.followTrajectory(PosttoStackP2);
            drive.servo0.setPosition(closed);
            sleep(300);
            Gripper(0,0.5);
            sleep(300);
            drive.followTrajectorySequence(StacktoPost1);
            Gripper(1,1);
            sleep(400);
            drive.servo0.setPosition(open);
            sleep(300);
            Gripper(0,0);
            sleep(300);
            drive.followTrajectorySequence(parkGreen);
            armMove(0);
            sleep(500);


        } else if (drive.colorR.blue()>drive.colorR.green() && drive.colorR.blue()>drive.colorR.red()){
            telemetry.addLine("Blue");
            drive.followTrajectory(toPost);
            Gripper(1,1);
            sleep(500);
            drive.servo0.setPosition(open);
            sleep(300);
            Gripper(0,0);
            sleep(200);
            drive.followTrajectory(PosttoStackP1);
            drive.followTrajectory(PosttoStackP2);
            drive.servo0.setPosition(closed);
            sleep(300);
            Gripper(0,0.5);
            sleep(300);
            drive.followTrajectorySequence(StacktoPost1);
            Gripper(1,1);
            sleep(400);
            drive.servo0.setPosition(open);
            sleep(300);
            Gripper(0,0);
            sleep(300);
            drive.followTrajectorySequence(parkBlue);
            armMove(0);
            sleep(500);
        }

        telemetry.update();
    }

    public void armMove(int target) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.arm.setTargetPosition(target);
        drive.arm1.setTargetPosition(target);
        drive.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.arm.setPower(1);
        drive.arm1.setPower(1);
    }

    public void Gripper(double grabber, double wrist) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.servoA.setPosition(grabber);
        drive.servoR.setPosition(wrist);
        drive.servoL.setPosition(wrist);
    }
}
