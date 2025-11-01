//package org.firstinspires.ftc.teamcode.OpModes;
//
///* Copyright (c) 2019 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
//import org.firstinspires.ftc.teamcode.Hardware.MSParams;
//import org.firstinspires.ftc.teamcode.Libs.MSMechOps;
//import org.firstinspires.ftc.teamcode.PinpointDrive;
//
////@Disabled
//@Autonomous(name = "Auto - SPECIMENS SWEEP", group = "Competition", preselectTeleOp = "RobotTeleOp")
//public class RRAUTOSWEEP extends LinearOpMode{
//
//    public static String TEAM_NAME = "Mouse Spit";
//    public static int TEAM_NUMBER = 11572;
//
//    public final static HWProfile2 robot = new HWProfile2();
//    public final static MSParams params = new MSParams();
//
//    public LinearOpMode opMode = this;
//    public MSMechOps mechOps;
//
//    //PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
//    PinpointDrive drive = null;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        //TODO: Initialize hardware
//        robot.init(hardwareMap, false);
//        mechOps = new MSMechOps(robot, opMode, params);
//        mechOps.restractStrings();
//
//        robot.servoSpice.setPosition(params.SPICE_CLOSE);
//        robot.servoClaw.setPosition(params.CLAW_CLOSE);
//        robot.servoBar.setPosition(params.Bar_Up);
//        robot.servoTwist.setPosition(params.TWIST_HORIZONTAL);
//        robot.servoWrist.setPosition(params.Wrist_Box);
//        robot.servoExtend.setPosition(params.Extend_IN);
//        robot.servoExtendRight.setPosition(params.ExtendRight_IN);
//        robot.servoBucket.setPosition(params.Bucket_Catch);
//
//        while (!isStopRequested() && !opModeIsActive()) {
//            // Wait for the DS start button to be touched.
//            telemetry.addData(">", "Touch Play to start OpMode");
//
//            telemetry.update();
//            robot.servoSpice.setPosition(params.SPICE_CLOSE);
//        }
//        //Game Play Button  is pressed
//        if (opModeIsActive() && !isStopRequested()) {
//            runAutonoumousMode();
//        }
//    }
//
//    //end runOpMode();
//
//    public void runAutonoumousMode() {
//        //Initialize Pose2d as desired
//        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
//        Pose2d specimenPrePreScoringPosition = new Pose2d(0, 0, 0);
//        Pose2d specimenPreScoringPosition = new Pose2d(0, 0, 0);
//        Pose2d specimenScoringPosition = new Pose2d(0, 0, 0);
//        Pose2d specimenScoringSlide = new Pose2d(0, 0, 0);
//        Pose2d sampleScoringPosition = new Pose2d(0, 0, 0);
//        Pose2d coloredSample1Position = new Pose2d(0, 0, 0);
//        Pose2d coloredSample2Position = new Pose2d(0, 0, 0);
//        Pose2d coloredSample3Position = new Pose2d(0, 0, 0);
//        Pose2d grabSpecimenPosition = new Pose2d(0, 0, 0);
//        Pose2d yellowSample1Position = new Pose2d(0, 0, 0);
//        Pose2d yellowSample2Position = new Pose2d(0, 0, 0);
//        Pose2d yellowSample3Position = new Pose2d(0, 0, 0);
//        Pose2d midwayPose0 = new Pose2d(0, 0, 0);
//        Pose2d midwayPose1 = new Pose2d(0, 0, 0);
//        Pose2d midwayPose2 = new Pose2d(0, 0, 0);
//        Pose2d midwayPose25 = new Pose2d(0, 0, 0);
//        Pose2d midwayPose3 = new Pose2d(0, 0, 0);
//        Pose2d midwayPose4 = new Pose2d(0, 0, 0);
//        Pose2d sweep1 = new Pose2d (0, 0, 0);
//        Pose2d sweep2 = new Pose2d (0, 0, 0);
//        Pose2d sweep3 = new Pose2d (0, 0, 0);
//
//        Pose2d parkPrepPose = new Pose2d(0, 0, 0);
//        Pose2d parkPose = new Pose2d(0, 0, 0);
//        double waitSecondsBeforeDrop = 0;
//        drive = new PinpointDrive(hardwareMap, initPose);
//
//        /*****************
//         * Set values for RoadRunner Pathing
//         */
//        specimenPrePreScoringPosition= new Pose2d(-15, 5, Math.toRadians(45));//old
//        specimenPreScoringPosition = new Pose2d(-18, -9, 0);
//        specimenScoringPosition = new Pose2d(-22, -12, 0);
//        specimenScoringSlide = new Pose2d(-30, -15, 0);
//        grabSpecimenPosition = new Pose2d(-1, 27.33, Math.toRadians(-180));
//        coloredSample1Position = new Pose2d(-5, 30, Math.toRadians(-90));
//        coloredSample2Position = new Pose2d(-35, -58, 90);
//        coloredSample3Position = new Pose2d(-35, -60, Math.toRadians(90));
//        midwayPose0 = new Pose2d(-21, 15, Math.toRadians(103)); //Before first pick old (-29,9)
//        midwayPose1 = new Pose2d(-28.33, 18.7, Math.toRadians(120.7)); //pick close to wall 35 x=-32
//        sweep1 = new Pose2d (-32.7, 21, 42.27);
//        midwayPose2 = new Pose2d(-21.6, 13, Math.toRadians(130)); //pick middle 25  .-27 x=-33
//        sweep2 = new Pose2d (-27.51, 13.92, 54);
//        midwayPose3 = new Pose2d(-15.59, 6, Math.toRadians(134.71));//pick first 15 x=-32
//        sweep3 = new Pose2d (-21.8, 13, 61);
//        midwayPose4 = new Pose2d(-14, 28.2, Math.toRadians(44.5)); // drop off
//
//        parkPose = new Pose2d(0, 9, Math.toRadians(90)); //-108
//
//        // Raise Arm to high bar scoring position
//
//        // TODO: Add code to release the sample and lower the arm
//        if (opModeIsActive()) robot.servoSpice.setPosition(params.CLAW_CLOSE);
//        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_CLIP_HIGH);
//        if (opModeIsActive()) robot.servoExtend.setPosition(params.Extend_OUT);
//        if (opModeIsActive()) robot.servoExtendRight.setPosition(params.ExtendRight_OUT);
//
//
//        telemetry.addData("x", drive.pose.position.x);
//        telemetry.addData("y", drive.pose.position.y);
//        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
//        telemetry.update();
//
//
//        // Drive to specimen scoring position
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                       // .strafeToLinearHeading(specimenPrePreScoringPosition.position, specimenPrePreScoringPosition.heading)
//                        .strafeToLinearHeading(specimenPreScoringPosition.position, specimenPreScoringPosition.heading)
//                        .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
//                        .build());
//
//        // Score specimen
//        telemetry.addData("x", drive.pose.position.x);
//        telemetry.addData("y", drive.pose.position.y);
//        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
//        telemetry.update();
//
//        // TODO: Add code to release the sample and lower the arm
//        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_CLIP_SCORE);
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(specimenScoringSlide.position, specimenScoringSlide.heading)
//                        .build());
//        if (opModeIsActive()) mechOps.openClaw();
//
//        telemetry.addData("x", drive.pose.position.x);
//        telemetry.addData("y", drive.pose.position.y);
//        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
//        telemetry.update();
//
//        //          Lower Lift
//        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_Sweep);
//        if (opModeIsActive()) mechOps.armout();
//        // Drive to color specimen Position
//        // Push Color Sample1 into the Observation area
//        // Drive to color sample3 Position
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(specimenPreScoringPosition.position,specimenPreScoringPosition.heading)
//                        .strafeToLinearHeading(midwayPose0.position, midwayPose0.heading)
////                            .strafeToLinearHeading(midwayPose4.position, midwayPose4.heading)
////                            .strafeToLinearHeading(coloredSample1Position.position, coloredSample1Position.heading)
////                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
////                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
////                            .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
//                        .build());
//
//        if (opModeIsActive()) robot.servoBar.setPosition(params.Bar_Down);
//        if (opModeIsActive()) robot.servoWrist.setPosition(params.Wrist_Sweep);
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(midwayPose3.position, midwayPose3.heading)
//                        .build());
//
//        if (opModeIsActive()) robot.servoBar.setPosition(params.Bar_Sweep);
//        //Turn to Sample 3 Drop
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(sweep3.position, sweep3.heading)
//                        .build());
//
//        if (opModeIsActive()) robot.servoBar.setPosition(params.Bar_Down);
//        //Turn to Sample Pick 2
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
//                        .build());
//
//        if (opModeIsActive()) robot.servoBar.setPosition(params.Bar_Sweep);
//        //Turn to Sample 2 Drop
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(sweep2.position, sweep2.heading)
//                        .build());
//
//        if (opModeIsActive()) robot.servoBar.setPosition(params.Bar_Down);
//
//
//        //Turn to Sample Pick 1
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
//                        .build());
//
//        if (opModeIsActive()) robot.servoBar.setPosition(params.Bar_Sweep);
//        //Turn to Sample 1 Drop
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(sweep1.position, sweep1.heading)
//                        .build());
//        if (opModeIsActive()) robot.servoBar.setPosition(params.Bar_Down);
//
//        if (opModeIsActive()) mechOps.armin();
//        // Grab the specimen 2
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
//                        .build());
//
//        Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .turnTo(Math.toRadians(-180))
//                            .lineToX(3)
//                            .build()  );
//
//        if (opModeIsActive()) robot.servoSpice.setPosition(params.SPICE_CLOSE);
//        safeWaitSeconds(0.1);
//
//        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_CLIP_HIGH);
//
//
//        // Drive to specimen scoring position
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        //.strafeToLinearHeading(specimenPrePreScoringPosition.position, specimenPrePreScoringPosition.heading)
//                        .strafeToLinearHeading(specimenPreScoringPosition.position, specimenPreScoringPosition.heading)
//                        .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
//                        .build());
//
//        // Score specimen
//
//        // TODO: Add code to release the sample and lower the arm
//        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_CLIP_SCORE);
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(specimenScoringSlide.position, specimenScoringSlide.heading)
//                        .build());
//        if (opModeIsActive()) mechOps.openClaw();
//        //          Lower Lift
//        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_MIN_LOW);
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
//                        .build());
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .turnTo(Math.toRadians(-180))
//                        .lineToX(3)
//                        .build()  );
//        // Grab the specimen 3
//        if (opModeIsActive()) robot.servoSpice.setPosition(params.SPICE_CLOSE);
//        safeWaitSeconds(0.1);
//
//        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_CLIP_HIGH);
//
//
//        // Drive to specimen scoring position
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        //.strafeToLinearHeading(specimenPrePreScoringPosition.position, specimenPrePreScoringPosition.heading)
//                        .strafeToLinearHeading(specimenPreScoringPosition.position, specimenPreScoringPosition.heading)
//                        .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
//                        .build());
//
//        // Score specimen
//
//        // TODO: Add code to release the sample and lower the arm
//        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_CLIP_SCORE);
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(specimenScoringSlide.position, specimenScoringSlide.heading)
//                        .build());
//        if (opModeIsActive()) mechOps.openClaw();
//
//        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_MIN_LOW);
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
//                        .build());
//
////here to stop 4th Spec
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .turnTo(Math.toRadians(-180))
//                        .lineToX(3)
//                        .build()  );
//        // Grab the specimen 4
//        if (opModeIsActive()) robot.servoSpice.setPosition(params.SPICE_CLOSE);
//        safeWaitSeconds(0.1);
//
//        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_CLIP_HIGH);
//
//
//        // Drive to specimen scoring position
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        //.strafeToLinearHeading(specimenPrePreScoringPosition.position, specimenPrePreScoringPosition.heading)
//                        .strafeToLinearHeading(specimenPreScoringPosition.position, specimenPreScoringPosition.heading)
//                        .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
//                        .build());
//
//        // Score specimen
//
//        // TODO: Add code to release the sample and lower the arm
//        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_CLIP_SCORE);
//        //      Actions.runBlocking(
// //               drive.actionBuilder(drive.pose)
// //                       .strafeToLinearHeading(specimenScoringSlide.position, specimenScoringSlide.heading)
// //                       .build());
//
//        if (opModeIsActive()) safeWaitSeconds(0.25);
//        if (opModeIsActive()) mechOps.openClaw();
//
//        if (opModeIsActive()) mechOps.raiseLift(params.LIFT_MIN_LOW);
//        if (opModeIsActive()) mechOps.armoutwinner();
//
//
//
//        //          Lower Lift
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                       // .strafeToLinearHeading(specimenPreScoringPosition.position,specimenPreScoringPosition.heading)
//                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
//                        .build());
//
//        telemetry.addData("x", drive.pose.position.x);
//        telemetry.addData("y", drive.pose.position.y);
//        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
//        telemetry.update();
//
//    }
//
//    /**
//     *
//     */
//
//    //method to wait safely with stop button working if needed. Use this instead of sleep
//    public void safeWaitSeconds(double time) {
//        ElapsedTime timer = new ElapsedTime(SECONDS);
//        timer.reset();
//        while (!isStopRequested() && timer.time() < time) {
//        }
//    }
//
//}   // end class
