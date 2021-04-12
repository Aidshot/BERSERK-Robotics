package org.firstinspires.ftc.teamcode.drive.opmode.BERSERK;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class AutoBv2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareBERSERK robot    = new HardwareBERSERK();
        robot.init(hardwareMap);

        double shooter_target_velo = 1800;
        double launch_angle = 0.178; //0.173
        double kicker_out = 0.7;
        double kicker_in = 0.2;
        double wobble_close = 0.18;
        double wobble_open = 0.6;
        double wobble_up = 0.6;
        double wobble_down = 0.2;
        long shootWait = 380;

        double webcam_right = 0.3;

        robot.webcam_servo.setPosition(webcam_right);

        waitForStart();

        robot.kicker.setPosition(kicker_out);
        Pose2d startPose = new Pose2d(-63.0,50, Math.toRadians(0.0));
        drive.setPoseEstimate(startPose);

        if (isStopRequested()) return;

        //   B AUTO TRAJECTORIES   //
        //SHOOT POSITION
        Trajectory B1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-25.0, 55.0), Math.toRadians(0.0))
                .addTemporalMarker(0.1, () -> {
                    robot.foldout_lift.setPower(-1);
                })
                .addTemporalMarker(1.8, () -> {
                    robot.foldout_lift.setPower(0);
                })
                .splineTo(new Vector2d(-3.0, 44.0), Math.toRadians(-3.0))
                .build();

        //WOBBLE B POSITION
        Trajectory B2 = drive.trajectoryBuilder(B1.end())
                .splineToLinearHeading(new Pose2d(29.0, 36.0, Math.toRadians(-90.0)), Math.toRadians(0.0))
                .build();

        //MOVE TOWARDS STACK
      //  Trajectory B3 = drive.trajectoryBuilder(B2.end(),true)
       //         .splineToLinearHeading(new Pose2d(-19.0, 42.0, Math.toRadians(180.0)), Math.toRadians(180.0))
      //          .build();

        //PICKUP WOBBLE
        Trajectory B4 = drive.trajectoryBuilder(B2.end(),true)
                .strafeRight(5)
                .splineToSplineHeading(new Pose2d(-19.0, 39.0, Math.toRadians(180.0)), Math.toRadians(180.0))
                .splineToSplineHeading( new Pose2d(-35.0, 30.0, Math.toRadians(135.0)), Math.toRadians(180))
                .splineToConstantHeading( new Vector2d(-40.0, 28.0), Math.toRadians(100),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(8, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(8))
                .build();

        //SHOOT
        Trajectory B5 = drive.trajectoryBuilder(B4.end())
                .splineToLinearHeading( new Pose2d(-6.0,44.0, Math.toRadians(7.0)), Math.toRadians(0.0))
                .build();

        //DROP WOBBLE
        Trajectory B6 = drive.trajectoryBuilder(B5.end())
                .splineToLinearHeading( new Pose2d(18.0, 45.0, Math.toRadians(-93.0)), Math.toRadians(0.0))
                .build();

        //PARK
        Trajectory B7 = drive.trajectoryBuilder(B6.end())
                .splineToLinearHeading( new Pose2d(6.0,22.0, Math.toRadians(-4.0)), Math.toRadians(-90.0))
                .build();

        // B AUTO //
        telemetry.addData("Stack:", "ONE");
        telemetry.update();

        //SET SERVOS
        robot.wobble_lift.setPosition(wobble_up);
        robot.wobble_claw.setPosition(wobble_close);
        robot.flap.setPosition(launch_angle);

        ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);

        //SHOOT POSITION
        drive.followTrajectory(B1);

        //SHOOT x 3
        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);

        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);

        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_out);

        //TURN OFF SHOOTER
        ((DcMotorEx) robot.shooter1).setVelocity(0);
        ((DcMotorEx) robot.shooter2).setVelocity(0);

        //WOBBLE B POSITION
        drive.followTrajectory(B2);

        //DROP WOBBLE 1
        robot.wobble_lift.setPosition(wobble_down);
        sleep(700);
        robot.wobble_claw.setPosition(wobble_open);
        sleep(200);

        //MOVE TOWARDS STACK
        robot.intake.setPower(0.8);
        robot.feeder_turn.setPower(1);
     //   drive.followTrajectory(B3);

        //PICKUP WOBBLE
        drive.followTrajectory(B4);
        robot.wobble_claw.setPosition(wobble_close);
        sleep(700);
        robot.wobble_lift.setPosition(wobble_up);
        sleep(190);

        //SHOOT POSITION
        ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);
        drive.followTrajectory(B5);

        //SHOOT X 1
        sleep(1500);

        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_out);

        //TURN OFF INTAKE AND SHOOTER
        robot.intake.setPower(0);
        robot.feeder_turn.setPower(0);
        ((DcMotorEx) robot.shooter1).setVelocity(0);
        ((DcMotorEx) robot.shooter2).setVelocity(0);

        //DROP WOBBLE 2
        drive.followTrajectory(B6);
        robot.wobble_lift.setPosition(wobble_down);
        sleep(700);
        robot.wobble_claw.setPosition(wobble_open);
        sleep(400);
        robot.wobble_lift.setPosition(wobble_up);

        //PARK
        drive.followTrajectory(B7);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
