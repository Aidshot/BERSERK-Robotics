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
public class AutoCv2 extends LinearOpMode {
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

        //Drop Wobble 1
        Trajectory C1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-25.0, 55.0), Math.toRadians(0.0))
                .splineTo(new Vector2d(51.0, 55.0), Math.toRadians(3.0))
                .build();

        //Shoot Powershots
        Trajectory C2 = drive.trajectoryBuilder(C1.end(),true)
                .splineToConstantHeading(new Vector2d(-5.0, 30.0), Math.toRadians(-90.0))
                .splineToConstantHeading(new Vector2d(-5.0, 8.0), Math.toRadians(-90.0),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(6, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(6))

                .addTemporalMarker(0.1, () -> {
                    robot.foldout_lift.setPower(-1);
                })
                .addTemporalMarker(1.8, () -> {
                    robot.foldout_lift.setPower(0);
                })
                //SHOOT
                .addTemporalMarker(3.1, () -> {
                    robot.kicker.setPosition(kicker_in);
                })
                .addTemporalMarker(3.5, () -> {
                    robot.kicker.setPosition(kicker_out);
                })

                //SHOOT
                .addTemporalMarker(4, () -> {
                    robot.kicker.setPosition(kicker_in);
                })
                .addTemporalMarker(4.5, () -> {
                    robot.kicker.setPosition(kicker_out);
                })

                //SHOOT
                .addTemporalMarker(5.05, () -> {
                    robot.kicker.setPosition(kicker_in);
                })
                .addTemporalMarker(5.6, () -> {
                    robot.kicker.setPosition(kicker_out);
                })

                //Pick Up Wobble 2
                .splineToSplineHeading(new Pose2d(-40.0, 12.0, Math.toRadians(45)), Math.toRadians(140.0))
                .splineToLinearHeading(new Pose2d(-43.0, 16.0, Math.toRadians(45)), Math.toRadians(140.0),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(40))
                .build();

        //Intake Stack
        Trajectory C3 = drive.trajectoryBuilder(C2.end())
                .splineToSplineHeading( new Pose2d(-34.0, 28.0, Math.toRadians(62.0)), Math.toRadians(62.0),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(80))
                .splineToConstantHeading( new Vector2d(-26.0, 43.0), Math.toRadians(62.0),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(10))
                .build();

        //Drop Wobble
        Trajectory C4 = drive.trajectoryBuilder(C3.end(),true)
                .splineToLinearHeading( new Pose2d(46.0, 50.0, Math.toRadians(4.0)), Math.toRadians(0.0))
                .build();

        //Shoot
        Trajectory C5 = drive.trajectoryBuilder(C4.end(),true)
                .splineToLinearHeading( new Pose2d(-3.0, 42.0, Math.toRadians(11.0)), Math.toRadians(180.0))
                .build();

        //Park
        Trajectory C6 = drive.trajectoryBuilder(C5.end())
                .splineToLinearHeading(  new Pose2d(6.0,22.0, Math.toRadians(8.0)), Math.toRadians(-90.0))
                .build();

        // C AUTO //
        telemetry.addData("Stack:", "FOUR");
        telemetry.update();

        //SET SERVOS
        robot.wobble_lift.setPosition(wobble_up);
        robot.wobble_claw.setPosition(wobble_close);
        robot.flap.setPosition(0.2);
       // ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);

        //WOBBLE C ZONE
        drive.followTrajectory(C1);
        ((DcMotorEx) robot.shooter1).setVelocity(1750);

        //DROP WOBBLE 1
        robot.wobble_lift.setPosition(wobble_down);
        sleep(400);
        robot.wobble_claw.setPosition(wobble_open);
        sleep(300);

        //SHOOT POWERSHOTS AND PICKUP WOBBLE
        drive.followTrajectory(C2);

        robot.wobble_claw.setPosition(wobble_close);
        sleep(700);
        robot.wobble_lift.setPosition(wobble_up);
        ((DcMotorEx) robot.shooter2).setVelocity(0);

        //INTAKE STACK
        robot.intake.setPower(0.8);
        robot.feeder_turn.setPower(1);
        drive.followTrajectory(C3);

        //DROP WOBBLE
        drive.followTrajectory(C4);
        robot.flap.setPosition(0.174);

        //DROP WOBBLE 2
        robot.wobble_lift.setPosition(wobble_down);
        sleep(400);
        robot.wobble_claw.setPosition(wobble_open);
        sleep(200);

        ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);

        //SHOOT POSITION
        drive.followTrajectory(C5);

        //SHOOT x 3
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
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_out);

        //TURN OFF SHOOTER
        robot.intake.setPower(0);
        robot.feeder_turn.setPower(0);
        ((DcMotorEx) robot.shooter1).setVelocity(0);
        ((DcMotorEx) robot.shooter2).setVelocity(0);

        //PARK
        drive.followTrajectory(C6);
        PoseStorage.currentPose = drive.getPoseEstimate();


    }
}
