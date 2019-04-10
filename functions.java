package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;
import static java.lang.StrictMath.cos;

public abstract class functions extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_Connection robot = new Hardware_Connection();

    static final double COUNTS_PER_MOTOR_NEVEREST40 = 1120;
    static final double COUNTS_PER_MOTOR_TETRIX = 1440;
    static final double DRIVE_GEAR_REDUCTION_NEVEREST40 = 1;
    static final double WHEEL_DIAMETER_CM = 10.5360;
    static final double PULLEY_DIAMETER_CM = 5;
    static final double STEER = 0.93; //friction coefficiant.
    static final int COUNTS_PER_CM_ANDYMARK_WHEEL = (int) ((COUNTS_PER_MOTOR_NEVEREST40 * DRIVE_GEAR_REDUCTION_NEVEREST40) / (WHEEL_DIAMETER_CM * Pi.getNumber()) * STEER);
    static final int COUNTS_PER_ANGLE_LIFT_ARM = (int)((COUNTS_PER_MOTOR_TETRIX/360) * 9);

    private ElapsedTime runtime = new ElapsedTime();


    double TurnPower = 0;


    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.05;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.05;     // Larger is more responsive, but also less stable

    public boolean ErrorOnArmOpen=false;

    public enum gyroDriveDirection {
        LEFTandRIGHT,
        FORWARDandBACKWARD,
        DIAGONALRIGHT,
        DIAGONALLEFT
    }


    public enum motorType {
        ARM,
        DRIVE,
        OPENING_SYSTEM,
        All
    }

    private enum GoldPos {
        Right,
        Left,
        Center,
        None;
    }


    public GoldPos goldPos = GoldPos.None;

    public void runOpMode() {
    }


    public void gyroDrive(double speed,
                          int distance,
                          double angle,
                          gyroDriveDirection direction) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        robot.drivingSetMode(RUN_USING_ENCODER);
        robot.drivingSetMode(STOP_AND_RESET_ENCODER);
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double backSpeed;
        double frontSpeed;
        angle += gyroGetAngle();


        if (direction == gyroDriveDirection.FORWARDandBACKWARD) {
            telemetry.addData("gyroDrive", "gyroDrive");
            telemetry.update();

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                distance = (int) (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
                newLeftFrontTarget = robot.left_front_motor.getCurrentPosition() + distance;
                newRightFrontTarget = robot.right_front_motor.getCurrentPosition() + distance;
                newRightBackTarget = robot.right_back_motor.getCurrentPosition() + distance;
                newLeftBackTarget = robot.left_back_motor.getCurrentPosition() + distance;

                // Set Target and Turn On RUN_TO_POSITION
                robot.left_front_motor.setTargetPosition(newLeftFrontTarget);
                robot.right_front_motor.setTargetPosition(newRightFrontTarget);
                robot.right_back_motor.setTargetPosition(newRightBackTarget);
                robot.left_back_motor.setTargetPosition(newLeftBackTarget);

                robot.drivingSetMode(RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(abs(speed), 0.0, 1.0);
                robot.fullDriving(speed, speed);

                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive()) {
                    if (!robot.left_back_motor.isBusy() || !robot.left_front_motor.isBusy() ||
                            !robot.right_back_motor.isBusy() || !robot.right_front_motor.isBusy()) {
                        robot.fullDriving(0, 0);
                        break;
                    }

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    leftSpeed = speed + steer;
                    rightSpeed = speed - steer;


                    robot.fullDriving(leftSpeed,rightSpeed);
                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Actual", "%7d:%7d", robot.left_front_motor.getCurrentPosition(), robot.right_front_motor.getCurrentPosition(), robot.right_back_motor.getCurrentPosition(), robot.left_back_motor.getCurrentPosition());
                    telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                    telemetry.update();
                }


                // Stop all motion;
                robot.fullDriving(0, 0);
                robot.drivingSetMode(RUN_USING_ENCODER);
            }

        } else if (direction == gyroDriveDirection.LEFTandRIGHT) {

            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                distance = -(distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
                newLeftFrontTarget = robot.left_front_motor.getCurrentPosition() - distance;
                newRightFrontTarget = robot.right_front_motor.getCurrentPosition() + distance;
                newRightBackTarget = robot.right_back_motor.getCurrentPosition() - distance;
                newLeftBackTarget = robot.left_back_motor.getCurrentPosition() + distance;

                // Set Target and Turn On RUN_TO_POSITION
                robot.left_front_motor.setTargetPosition(newLeftFrontTarget);
                robot.right_front_motor.setTargetPosition(newRightFrontTarget);
                robot.right_back_motor.setTargetPosition(newRightBackTarget);
                robot.left_back_motor.setTargetPosition(newLeftBackTarget);

                robot.left_back_motor.setMode(RUN_TO_POSITION);
                robot.left_front_motor.setMode(RUN_TO_POSITION);
                robot.right_back_motor.setMode(RUN_TO_POSITION);
                robot.right_front_motor.setMode(RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(abs(speed), 0.0, 1.0);
                robot.driveToLEFTandRIGHT(speed, speed);


                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive() && (robot.left_back_motor.isBusy() && robot.left_front_motor.isBusy() &&
                        robot.right_back_motor.isBusy() && robot.right_front_motor.isBusy())) {

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance > 0) {
                        steer *= -1.0;
                    }
                    frontSpeed = speed + steer;
                    backSpeed = speed - steer;

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(abs(backSpeed), abs(frontSpeed));
                    if (max > 1.0) {
                        backSpeed /= max;
                        frontSpeed /= max;
                    }


                    robot.driveToLEFTandRIGHT(backSpeed, frontSpeed);


                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Actual", "%7d:%7d", robot.left_front_motor.getCurrentPosition(), robot.right_front_motor.getCurrentPosition(), robot.left_back_motor.getCurrentPosition(), robot.right_back_motor.getCurrentPosition());
                    telemetry.update();
                }


                // Stop all motion;
                robot.fullDriving(0, 0);

            }
        } else if (direction == gyroDriveDirection.DIAGONALLEFT) {
            distance = (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
            newRightBackTarget = robot.right_back_motor.getCurrentPosition() + distance;
            newLeftFrontTarget = robot.left_front_motor.getCurrentPosition() + distance;

            // Set Target and Turn On RUN_TO_POSITION
            robot.right_back_motor.setTargetPosition(newRightBackTarget);
            robot.left_front_motor.setTargetPosition(newLeftFrontTarget);

            robot.right_back_motor.setMode(RUN_TO_POSITION);
            robot.left_front_motor.setMode(RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(abs(speed), 0.0, 1.0);
            robot.driveToLEFTandRIGHT(speed, speed);

            if (opModeIsActive()) {
                distance = (int) (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
                if (distance < 0) {
                    speed = -speed;
                }
            }
            while (opModeIsActive() && abs(robot.left_front_motor.getCurrentPosition()) < abs(robot.left_front_motor.getTargetPosition())
                    && abs(robot.right_back_motor.getCurrentPosition()) < abs(robot.right_back_motor.getTargetPosition())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(abs(leftSpeed), abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.right_front_motor.setPower(leftSpeed);
                robot.left_back_motor.setPower(-rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newRightBackTarget, newLeftFrontTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.right_front_motor.getCurrentPosition(), robot.left_back_motor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }
            // Stop all motion;
            robot.fullDriving(0, 0);
            robot.drivingSetMode(RUN_USING_ENCODER);
        }
        else if (direction == gyroDriveDirection.DIAGONALRIGHT) {
            distance = (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
            newRightFrontTarget = robot.right_front_motor.getCurrentPosition() + distance;
            newLeftBackTarget = robot.left_back_motor.getCurrentPosition() + distance;

            // Set Target and Turn On RUN_TO_POSITION
            robot.right_front_motor.setTargetPosition(newRightFrontTarget);
            robot.left_back_motor.setTargetPosition(newLeftBackTarget);

            robot.left_back_motor.setMode(RUN_TO_POSITION);
            robot.right_front_motor.setMode(RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(abs(speed), 0.0, 1.0);
            robot.driveToLEFTandRIGHT(speed, speed);

            if (opModeIsActive()) {
                distance = (int) (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
                if (distance < 0) {
                    speed = -speed;
                }
            }
            while (opModeIsActive() && abs(robot.left_back_motor.getCurrentPosition()) < abs(robot.left_back_motor.getTargetPosition())
                    && abs(robot.right_front_motor.getCurrentPosition()) < abs(robot.right_front_motor.getTargetPosition())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(abs(leftSpeed), abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.left_front_motor.setPower(leftSpeed);
                robot.right_back_motor.setPower(-rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newRightFrontTarget, newLeftBackTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.right_front_motor.getCurrentPosition(), robot.left_back_motor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }
            // Stop all motion;
            robot.fullDriving(0, 0);
            robot.drivingSetMode(RUN_USING_ENCODER);
        }
    }
    public void gyroTurn(double speed, double angle) {
        angle += gyroGetAngle();

        robot.drivingSetMode(RUN_USING_ENCODER);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
        }
    }


    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            leftSpeed  = speed * steer;
            rightSpeed   = -leftSpeed;
        }

        // Send desired speeds to motors.
        robot.fullDriving(leftSpeed,rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    // Calculate error in -179 to +180 range.
    public double getError(double targetAngle) {

        double robotError;
        double angle = gyroGetAngle();
        robotError = targetAngle - angle;

        while (robotError > 180 && opModeIsActive()) {
            robotError -= 360;
        }
        while (robotError <= -180 && opModeIsActive()) {
            robotError += 360;
        }
        return robotError;
    }


    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public GoldPos GetGoldMineralPosition(){
        return GoldPos.Center;
    }




    public void climbDown(){
        armOpeningEncoder(1, 10);
        robot.arm_motors(0);
        runtime.reset();
        while(runtime.milliseconds() < 500 && opModeIsActive()){
            robot.arm_motors(0.2);
        }
        robot.arm_motors(0);
        gyroDrive(1, -5, 0, gyroDriveDirection.FORWARDandBACKWARD);
        gyroDrive(1, 7, 0, gyroDriveDirection.LEFTandRIGHT);

    }


    public void mineralToLanderCrater() {
        robot.arm_motors(0.5);
        gyroDrive(1,-70,15, gyroDriveDirection.LEFTandRIGHT);
        armEncoder(1,3000);
        robot.arm_motors(0.25);
        armOpeningEncoder(1,45);
        robot.mineral_keeper_servo.setPosition(robot.OPEN);
        armEncoder(1,-3000);
    }

    public float gyroGetAngle() {
        //telemetry.addData("y: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("z: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).secondAngle);
        //telemetry.addData("x: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).thirdAngle);
        return robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public void putTeamMarker() {
        armEncoder(1,-20);
        gyroDrive(1,70,0, gyroDriveDirection.LEFTandRIGHT);
        gyroTurn(1,-45);
        gyroDrive(1,45,0, gyroDriveDirection.LEFTandRIGHT);
        //pickUpMineral();
        gyroDrive(1,-5,0, gyroDriveDirection.LEFTandRIGHT);
        armOpeningEncoder(1,-10);
        armEncoder(1,-20);
        sleep(100);
        gyroDrive(1,-130,0, gyroDriveDirection.FORWARDandBACKWARD);
        robot.team_marker_servo.setPosition(0);
        gyroDrive(1,30,0, gyroDriveDirection.FORWARDandBACKWARD);
        robot.team_marker_servo.setPosition(1);
        gyroDrive(1,100,0, gyroDriveDirection.FORWARDandBACKWARD);
        armEncoder(1,20);
        armOpeningEncoder(1,30);

    }

    public void pickUpMineral(){
        armEncoder(1,-7);
        armOpeningEncoder(1,20);
        robot.arm_motors(0.15);
        armOpeningEncoder(1,-30);
        robot.arm_motors(0);
    }

    public void sampling(){
        gyroDrive(1,10,0, gyroDriveDirection.FORWARDandBACKWARD);
        switch(goldPos){
            case Right:
                telemetry.addData("Pos","right");
                telemetry.update();
                sleep(100);
                gyroDrive(1, 80, 0, gyroDriveDirection.DIAGONALRIGHT);
                telemetry.addData("take","מינרל");
                telemetry.update();
                sleep(100);
                takeMineral();
                gyroDrive(1, -30, 0, gyroDriveDirection.LEFTandRIGHT);
            case Left:
                telemetry.addData("Pos","Left");
                telemetry.update();
                sleep(100);
                gyroDrive(1, 80, 0, gyroDriveDirection.DIAGONALLEFT);
                telemetry.addData("take","מינרל");
                telemetry.update();
                sleep(100);
                takeMineral();
            case Center:
                telemetry.addData("Pos","Center");
                telemetry.update();
                sleep(100);
                gyroDrive(1 , 45, 0, gyroDriveDirection.FORWARDandBACKWARD);
                telemetry.addData("take","מינרל");
                telemetry.update();
                sleep(100);
                takeMineral();

        }
    }
    public void armOpeningEncoder(double speed, double distance) {
        robot.arm_opening_system.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm_opening_system.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int StartPos=robot.arm_opening_system.getCurrentPosition();


        int Target = (int) (distance * (COUNTS_PER_MOTOR_TETRIX / (Pi.getNumber() * PULLEY_DIAMETER_CM)));
        Target += robot.arm_opening_system.getCurrentPosition();
        Target *= -1;

        robot.arm_opening_system.setTargetPosition(Target);
        robot.arm_opening_system.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        while (!ErrorOnMotor(robot.arm_opening_system, StartPos) &&
                abs(Target) > abs(robot.arm_opening_system.getCurrentPosition()) &&
                opModeIsActive()) {
            telemetry.addData("current:", robot.arm_opening_system.getCurrentPosition());
            telemetry.addData("power:", robot.arm_opening_system.getPower());
            telemetry.addData("target:", Target);
            telemetry.update();
            if (Target > 0) {
                robot.arm_opening_system.setPower(speed);
                robot.arm_opening_system_2.setPower(speed);
            }
            else  {
                robot.arm_opening_system.setPower(-speed);
                robot.arm_opening_system_2.setPower(-speed);
            }
        }
        robot.arm_opening_system.setPower(0);
    }

    public void armEncoder(double speed, double Target) {
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm_motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Target = Target * COUNTS_PER_ANGLE_LIFT_ARM;
        robot.arm_motor_2.setTargetPosition((int)Target);
        telemetry.addData("target", Target);
        telemetry.addData("current", robot.arm_motor_2.getCurrentPosition());
        telemetry.update();
        int StartPos = robot.arm_motor_2.getCurrentPosition();
        while (abs(robot.arm_motor_2.getCurrentPosition()) < abs((int) Target) && !ErrorOnMotor(robot.arm_motor_2,StartPos) && opModeIsActive()) {
            telemetry.addData("current:", robot.arm_motor_2.getCurrentPosition());
            telemetry.addData("target:", Target);
            telemetry.update();
            robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (Target > robot.arm_motor_2.getCurrentPosition()) {
                robot.arm_motors(speed);
            }
            if (Target < robot.arm_motor_2.getCurrentPosition()) {
                robot.arm_motors(-speed);
            }
            robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("current1:", robot.arm_motor_2.getCurrentPosition());
            telemetry.addData("target1:", Target);
            telemetry.update();
        }
        robot.arm_motors(0);
    }

    public void samplingDepot(){
        switch(goldPos){
            case Right:
                gyroTurn(1,30);
                takeMineral();
                gyroTurn(1,-30);
            case Left:
                gyroTurn(1,-30);
                takeMineral();
                gyroTurn(1,30);
            case Center:
                gyroDrive(1 , -10, 0, gyroDriveDirection.LEFTandRIGHT);
                takeMineral();
        }

    }
    public void takeMineral(){
        armEncoder(1,-55);
        stopArmMotors();
        inputMineral();
        armOpeningEncoder(1,20);
        switch (goldPos) {
            case Center:
                gyroDrive(1, -30, 0, gyroDriveDirection.FORWARDandBACKWARD);
                gyroDrive(1, 15, 0, gyroDriveDirection.FORWARDandBACKWARD);
                robot.arm_motors(-0.1);
                gyroDrive(1, 10, 0, gyroDriveDirection.FORWARDandBACKWARD);
                armOpeningEncoder(1,10);
            case Left:
                gyroDrive(1, -35, 0, gyroDriveDirection.DIAGONALLEFT);
                gyroDrive(1, 25, 0, gyroDriveDirection.DIAGONALLEFT);
                robot.arm_motors(-0.1);
                gyroDrive(1, 17, 0, gyroDriveDirection.DIAGONALLEFT);
                robot.arm_motors(0);
                armOpeningEncoder(1,10);
                gyroDrive(1, -45, 0, gyroDriveDirection.DIAGONALLEFT);
                armEncoder(1,85);
                gyroDrive(1, -30, 0, gyroDriveDirection.LEFTandRIGHT);

            case Right:
                gyroDrive(1, -35, 0, gyroDriveDirection.DIAGONALRIGHT);
                gyroDrive(1, 25, 0, gyroDriveDirection.DIAGONALRIGHT);
                robot.arm_motors(-0.1);
                gyroDrive(1, 17, 0, gyroDriveDirection.DIAGONALRIGHT);
                robot.arm_motors(0);
                armOpeningEncoder(1,10);
                gyroDrive(1, -45, 0, gyroDriveDirection.DIAGONALRIGHT);
                armEncoder(1,40);
                gyroDrive(1, 30, 0, gyroDriveDirection.LEFTandRIGHT);

        }
        robot.arm_motors(0);
        armToLander();

    }
    public void armToLander(){
        if(goldPos == GoldPos.Center){
            gyroDrive(1, -30, 0, gyroDriveDirection.FORWARDandBACKWARD);
            armEncoder(1,85);
        }
        else{
            armEncoder(1,45);
        }
        armOpeningEncoder(1,-15);
        armEncoder(0.7,20);
        robot.mineral_keeper_servo.setPosition(robot.OPEN);
        inputMineral();
        sleep(750);
        armEncoder(1,-40);
        armOpeningEncoder(1,-10);

    }
    public void putTeamMarkerDepot() {
        gyroDrive(1,35,0,gyroDriveDirection.FORWARDandBACKWARD);
        robot.arm_motors(-0.1);
        armOpeningEncoder(1,10);
        robot.arm_motors(0);
        armOpeningEncoder(1,40);
        armOpeningEncoder(1,-50);
        robot.arm_motors(0);
    }

    public void goToCraterDepot() {
        gyroDrive(11,-100,0,gyroDriveDirection.LEFTandRIGHT);
        gyroTurn(1,45);
        gyroDrive(1,-50,0,gyroDriveDirection.FORWARDandBACKWARD);

    }


    public boolean ErrorOnMotor(DcMotor motor,int StartPos){
        boolean ErrorMotor = false;
        //if(runtime.seconds()>0.5 && robot.IntInRange(StartPos-30,StartPos+30,motor.getCurrentPosition())){
        //  ErrorMotor=true;
        //}
        return ErrorMotor;
    }

    public void inputMineral(){
        robot.arm_collecting_system.setPower(-0.8);
    }
    public void stopMineral(){
        robot.arm_collecting_system.setPower(0);
    }
    public void OutPutMineral(){
        runtime.reset();
        robot.arm_collecting_system.setPower(0.8);
        while(runtime.seconds()<1 && opModeIsActive()){
            telemetry.addData(">","OUTPUT mineral");
            telemetry.update();
        }
        robot.arm_collecting_system.setPower(0);
    }
    public void stopArmMotors(){
        robot.arm_motor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public boolean MotorInPosition(DcMotor motor){
        return motor.getCurrentPosition() > motor.getTargetPosition();
    }
}