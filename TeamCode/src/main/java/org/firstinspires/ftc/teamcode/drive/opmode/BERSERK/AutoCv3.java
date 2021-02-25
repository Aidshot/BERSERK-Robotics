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
public class AutoCv3 extends LinearOpMode {
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
                .splineTo(new Vector2d(51.0, 50.0), Math.toRadians(0.0))
                .build();

        //SHOOT POWERSHOTS AND GRAB WOBBLE 2
        Trajectory C2 = drive.trajectoryBuilder(C1.end())
                //Approach Powershots
                .splineToConstantHeading(new Vector2d(-5.0, 20.0), Math.toRadians(-90.0))
                .addDisplacementMarker(() -> {
                    ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);
                })
                //Strafe and Shoot
                .splineToConstantHeading(new Vector2d(-5.0, -2.0), Math.toRadians(-90.0),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(8, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(8))

                //Shoot
                .addSpatialMarker(new Vector2d(-5, 18), () -> {
                    robot.kicker.setPosition(kicker_in);
                })
                .addSpatialMarker(new Vector2d(-5, 14), () -> {
                    robot.kicker.setPosition(kicker_out);
                })

                //Shoot
                .addSpatialMarker(new Vector2d(-5, 10), () -> {
                    robot.kicker.setPosition(kicker_in);
                })
                .addSpatialMarker(new Vector2d(-5, 6), () -> {
                    robot.kicker.setPosition(kicker_out);
                })

                //Shoot
                .addSpatialMarker(new Vector2d(-5, 2), () -> {
                    robot.kicker.setPosition(kicker_in);
                })
                .addSpatialMarker(new Vector2d(-5, -2), () -> {
                    robot.kicker.setPosition(kicker_out);
                })

                //SHOOT
             //   .addTemporalMarker(0.1, () -> {
             //       robot.kicker.setPosition(kicker_in);
             //   })
             //   .addTemporalMarker(0.5, () -> {
             //       robot.kicker.setPosition(kicker_out);
             //   })

                //SHOOT
             //   .addTemporalMarker(1, () -> {
             //       robot.kicker.setPosition(kicker_in);
             //   })
             //   .addTemporalMarker(1.5, () -> {
             //       robot.kicker.setPosition(kicker_out);
             //   })

                //SHOOT
              //  .addTemporalMarker(2, () -> {
              //      robot.kicker.setPosition(kicker_in);
              //  })
              //  .addTemporalMarker(2.5, () -> {
              //      robot.kicker.setPosition(kicker_out);
              //  })

                //Approach Wobble 2
                .splineToSplineHeading(new Pose2d(-30.0, 5.0, Math.toRadians(45)), Math.toRadians(140.0))
                .splineToLinearHeading(new Pose2d(-35.0, 10.0, Math.toRadians(45)), Math.toRadians(140.0),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(10))
                .build();

        //STRAFE AND SHOOT
        Trajectory C3 = drive.trajectoryBuilder(C2.end())
                .splineTo(new Vector2d(-10.0, 20.0), Math.toRadians(0.0))
                .splineTo(new Vector2d(51.0, 37.0), Math.toRadians(0.0))
                .build();

        //DROP WOBBLE 2
        Trajectory C4 = drive.trajectoryBuilder(C3.end())
                .splineToSplineHeading(new Pose2d(-15.0, 38.0, Math.toRadians(180.0)), Math.toRadians(180.0))
                .build();

        //MOVE TOWARDS STACK
        Trajectory C5 = drive.trajectoryBuilder(C4.end())
                .splineTo(new Vector2d(-30.0, 38.0), Math.toRadians(180.0),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(10))
                .splineToSplineHeading(new Pose2d(-2.0, 38.0, Math.toRadians(0.0)), Math.toRadians(180.0))
                .build();

        //INTAKE STACK
        Trajectory C6 = drive.trajectoryBuilder(C5.end())
                .forward(5.0)
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

        drive.followTrajectory(C1);
        drive.followTrajectory(C2);
        ((DcMotorEx) robot.shooter1).setVelocity(0);
        drive.followTrajectory(C3);
        drive.followTrajectory(C4);
        drive.followTrajectory(C5);
        drive.followTrajectory(C6);
    }
}
