package org.firstinspires.ftc.teamcode.drive.opmode.BERSERK;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
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

        //DROP WOBBLE C
        Trajectory C1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-30.0, 55.0), Math.toRadians(0.0))
                .splineTo(new Vector2d(55.0, 50.0), Math.toRadians(0.0))
                .build();

        //SHOOT POWERSHOTS AND GRAB WOBBLE 2
        Trajectory C2 = drive.trajectoryBuilder(C1.end(),true)
                .splineToConstantHeading(new Vector2d(-5.0, 18.0), Math.toRadians(-90.0))
                .splineToConstantHeading(new Vector2d(-5.0, 18.0), Math.toRadians(-90.0))
                .splineToSplineHeading(new Pose2d(-5.0, 18.0, Math.toRadians(45)), Math.toRadians(140.0))
                .splineToLinearHeading(new Pose2d(-5.0, 18.0, Math.toRadians(45)), Math.toRadians(140.0))
                .build();

        //STRAFE AND SHOOT
        Trajectory C3 = drive.trajectoryBuilder(C2.end(),true)
                .splineToConstantHeading(new Vector2d(-5.0, 0.0), Math.toRadians(-90.0))

                //SHOOT
                .addTemporalMarker(0.1, () -> {
                    robot.kicker.setPosition(kicker_in);
                })
                .addTemporalMarker(0.5, () -> {
                    robot.kicker.setPosition(kicker_out);
                })

                //SHOOT
                .addTemporalMarker(1, () -> {
                    robot.kicker.setPosition(kicker_in);
                })
                .addTemporalMarker(1.5, () -> {
                    robot.kicker.setPosition(kicker_out);
                })

                //SHOOT
                .addTemporalMarker(2, () -> {
                    robot.kicker.setPosition(kicker_in);
                })
                .addTemporalMarker(2.5, () -> {
                    robot.kicker.setPosition(kicker_out);
                })

                //MOVE TO WOBBLE 2
                .splineToSplineHeading(new Pose2d (-30.0, 5.0, Math.toRadians(45.0)), Math.toRadians(140.0))

                .splineToSplineHeading(new Pose2d (-35.0, 10.0, Math.toRadians(45.0)), Math.toRadians(140.0),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(4, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(4))
                .build();

        //DROP WOBBLE 2
        Trajectory C4 = drive.trajectoryBuilder(C3.end())
                .splineToSplineHeading(new Pose2d (55.0, 40.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .build();

        //MOVE TOWARDS STACK
        Trajectory C5 = drive.trajectoryBuilder(C4.end(),true)
                .splineToLinearHeading(new Pose2d(0.0, 38.0, Math.toRadians(180.0)), Math.toRadians(180.0))
                .forward(12,
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(80))
                .build();

        //INTAKE STACK (Slow Constraints)
        Trajectory C6 = drive.trajectoryBuilder(C5.end())
                .forward(10,
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //SHOOT POSITION
        Trajectory C7 = drive.trajectoryBuilder(C6.end(),true)
                .splineToLinearHeading( new Pose2d(-3.0, 38.0, Math.toRadians(-3.0)), Math.toRadians(180))
                .build();

        //PARK
        Trajectory C8 = drive.trajectoryBuilder(C7.end())
                .forward(5)
                .build();

        // C AUTO //
        telemetry.addData("Stack:", "FOUR");
        telemetry.update();

        robot.intake.setPower(0.8);
        sleep(500);
        robot.intake.setPower(0.0);

        //SET SERVOS
        robot.wobble_lift.setPosition(wobble_up);
        robot.wobble_claw.setPosition(wobble_close);
        launch_angle= 0.2;
        robot.flap.setPosition(launch_angle);

        //Drop Wobble 1
        drive.followTrajectory(C1);

        robot.wobble_lift.setPosition(wobble_down);
        sleep(400);
        robot.wobble_claw.setPosition(wobble_open);
        sleep(200);

        //Move to Powershots
        drive.followTrajectory(C2);
        //Strafe and Shoot
        drive.followTrajectory(C3);

        //Pick up Wobble 2
        robot.wobble_claw.setPosition(wobble_close);
        sleep(800);
        robot.wobble_lift.setPosition(wobble_up);
        sleep(800);

        //Drop Wobble 2
        drive.followTrajectory(C4);

        robot.wobble_lift.setPosition(wobble_down);
        sleep(400);
        robot.wobble_claw.setPosition(wobble_open);
        sleep(200);

        drive.followTrajectory(C5);

        robot.wobble_claw.setPosition(wobble_close);
        robot.wobble_lift.setPosition(wobble_up);

        drive.followTrajectory(C6);

        launch_angle= 0.178;
        robot.flap.setPosition(launch_angle);

        drive.followTrajectory(C7);

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

        drive.followTrajectory(C8);
    }
}
