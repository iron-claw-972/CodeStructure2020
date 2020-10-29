package frc.robot.util;

import java.util.ArrayList;

import org.ejml.simple.*;
import org.mockito.internal.junit.MismatchReportingTestListener;

public class SplineFollower {

    SimpleMatrix invSplMat;
    // Inverse of linear systems needed to solve for quintic spline terms, faster
    // solution
    SimpleMatrix splDerMat;
    // Multiplying a spline vector returns the derivative vector of the spline

    //TODO: Throw exceptions for wrong input

    double du;
    double ds;

    public SimpleMatrix[][] cubSplMat;
    //0: Spline vectors for X sequentially
    //1: Spline vectors for Y sequentially

    public double[][] paramToArc;
    //0: Spline input parameter corresponding to arclength
    //1: Arclength, incrementing sequentially in ds

    public double[][] velocityProfile;
    //0: Spline input parameter corresponding to velocity
    //1: Velocity of the robot along spline

    public double[][] stateProfile;
    //0: Time corresponding to state
    //1: Curvature [- = left, + = right, 0 = straight]
    //2: Velocity

    public SplineFollower(double[][] pathPoints, double[][] pathVel) {

        /*if (pathPoints[0].length != pathVel[0].length) {
            throw new InterruptedException();
            "Number of path points and velocity vectors do not match"
        }

        if (pathPoints[0].length != 2 || pathVel.length != 2) {
            // throw new Exception("pathPoints or pathVel is not in 2 dimensions");
        }*/
        //TODO: Fix this exception gore

        du = 0.0001;
        ds = 0.01;

        // Describes P(t) = ax^3 + bx^2 + cx + d
        double[][] splineMat = new double[][] { 
            { 0, 0, 0, 1 }, // P(0) = d
            { 1, 1, 1, 1 }, // P(1) = a + b + c + d
            { 0, 0, 1, 0 }, // P'(0) = c
            { 3, 2, 1, 0 } // P'(1) = 3a + 2b + c
        };
        SimpleMatrix splineMatrix = new SimpleMatrix(splineMat);
        invSplMat = splineMatrix.invert();

        double[][] splineDMat = new double[][] { 
            { 0, 0, 0, 0 }, 
            { 3, 0, 0, 0 }, 
            { 0, 2, 0, 0 }, 
            { 0, 0, 1, 0 } 
        };
        splDerMat = new SimpleMatrix(splineDMat);

        generateSplines(pathPoints, pathVel);
        generateParamToArc();
        generateVelocityProfile();
        generateStateProfile();
    }

    //COMPLETE
    public void generateSplines(double[][] pathPoints, double[][] pathVel) {

        System.out.println("Generating Splines...");

        int numberOfPoints = pathPoints[0].length;

        double[][] XYVelocities = new double[2][numberOfPoints];

        for (int i = 0; i < numberOfPoints; i++) {
            double radians = (pathVel[0][i] * Math.PI / 180);
            XYVelocities[0][i] = pathVel[1][i] * Math.cos(radians);
            XYVelocities[1][i] = pathVel[1][i] * Math.sin(radians);
        }

        SimpleMatrix[][] cubSplMat_ = new SimpleMatrix[2][numberOfPoints - 1];
        // first index: 0 = x, 1 = y
        // second: sequential order of splines
        // third: coefficients

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < cubSplMat_.length; j++) {
                double[][] splProbMat = new double[][] { 
                    { pathPoints[i][j] }, 
                    { pathPoints[i][j + 1] }, 
                    { XYVelocities[i][j] }, 
                    { XYVelocities[i][j + 1] } 
                };

                SimpleMatrix splineProblem = new SimpleMatrix(splProbMat);

                SimpleMatrix splineSolution = invSplMat.mult(splineProblem);
                cubSplMat_[i][j] = splineSolution;
            }
        }

        System.out.println("Completed Splines");

        cubSplMat = cubSplMat_;
    }

    //COMPLETE
    public void generateParamToArc() {

        System.out.println("Generating Input Parameter to Arclength Map...");

        ArrayList<Double> params = new ArrayList<>();
        ArrayList<Double> arcs = new ArrayList<>();

        double arcLength = 0;
        double lastArcLength = 0;

        params.add(0.0);
        arcs.add(arcLength);

        SimpleMatrix currPos = getPosition(0);
        SimpleMatrix lastPos = currPos;

        int arcsCounted = 0;
        int lastCounted = 0;

        for (double u = 0; u < cubSplMat[0].length; u += du) {

            SimpleMatrix posDiff = currPos.minus(lastPos);
            //System.out.println(posDiff);
            arcLength += vectorMag(posDiff);
            //System.out.println(arcLength);

            lastPos = currPos;
            currPos = getPosition(u);

            arcsCounted = (int)(arcLength / ds);
            int arcsDiff = arcsCounted - lastCounted;

            if (arcsDiff > 0) {

                double splitDt = du/arcsDiff;
                double splitDs = (arcLength - lastArcLength)/arcsDiff;
                double lastTime = u - du;

                //If the du resolution is too low, approximates by splitting apart equally
                for(int i = 1; i <= arcsDiff; i++) {
                    params.add(lastTime + i * splitDt);
                    arcs.add(lastArcLength + i * splitDs);
                }
            }

            lastCounted = arcsCounted;
            lastArcLength = arcLength;
        }

        double[] timeArray = doubleListToArray(params);
        double[] arcArray = doubleListToArray(arcs);

        paramToArc = new double[][] { 
            timeArray, 
            arcArray 
        };

        System.out.println("Completed Input Parameter to Arclength Map");
    }

    public void generateVelocityProfile() {

        System.out.println("Generating Velocity Profile...");

        double[][] velocityProfile_ = new double[2][paramToArc[0].length];
        // Maximum Lateral Acceleration (accounting for curvature), Maximum Velocity
        // Limit (Also changes with curvature due to discrete motors), velocity
        // continuity (max acceleration)

        //Establishing impossibly ideal initial parameters
        for (int i = 0; i < velocityProfile_[0].length; i++) {

            velocityProfile_[0][i] = paramToArc[0][i];
            velocityProfile_[1][i] = Context.MAX_MOTOR_SPEED;
        }

        //Impose initial condition
        velocityProfile_[1][0] = 0;
        velocityProfile_[1][velocityProfile_[0].length - 1] = 0;

        //Impose constraints
        velocityProfile_ = imposeIsolConstraints(velocityProfile_);
        velocityProfile_ = imposeContConstraints(velocityProfile_);

        System.out.println("Completed Velocity Profile");

        velocityProfile = velocityProfile_;
    }

    public double[][] imposeIsolConstraints(double[][] velocityProfile) {

        //Imposing absolute maximum constraints
        for (int i = 0; i < velocityProfile[0].length; i++) {

            int splineIndex = (int) (paramToArc[0][i]);
            double splineTime = paramToArc[0][i] - splineIndex;

            double K = curvature(paramToArc[0][i]);

            //The two maximum constraints we impose are 
            double maxFricVel = Math.sqrt(Context.MAX_LAT_FRICTION / (Context.ROBOT_MASS * Math.abs(K)));
            double maxTransVel = Context.MAX_MOTOR_SPEED / (1 + Math.abs(K) * Context.ROBOT_WIDTH / 2);

            velocityProfile[1][i] = Math.min(velocityProfile[1][i], Math.min(maxFricVel, maxTransVel));
        }

        return velocityProfile;
    }

    public double[][] imposeContConstraints(double[][] velocityProfile) {

        //Forward constraining
        for (int i = 1; i < velocityProfile[0].length; i++) {

            double lastSpeed = velocityProfile[1][i-1];

            double K = curvature(velocityProfile[0][i]);
            double maxAccel = Context.MAX_MOTOR_ACCEL / (1 + Math.abs(K) * Context.ROBOT_WIDTH / 2);

            double forwSpeed = Math.sqrt(lastSpeed * lastSpeed + 2 * maxAccel * ds);

            //System.out.println(i + " FORW: " + forwSpeed + " MAX: " + velocityProfile[1][i]);

            velocityProfile[1][i] = Math.min(velocityProfile[1][i], forwSpeed);
        }

        //Backward constraining
        for (int i = velocityProfile[0].length - 2; i >= 0; i--) {

            double nextSpeed = velocityProfile[1][i+1];

            double K = curvature(velocityProfile[0][i]);
            double maxAccel = Context.MAX_MOTOR_ACCEL / (1 + Math.abs(K) * Context.ROBOT_WIDTH / 2);

            double backSpeed = Math.sqrt(nextSpeed * nextSpeed + 2 * maxAccel * ds);

            //System.out.println(i + " BACK: " + backSpeed + " MAX: " + velocityProfile[1][i]);

            velocityProfile[1][i] = Math.min(velocityProfile[1][i], backSpeed);
        }

        return velocityProfile;
    }

    //Change to state profile
    public void generateStateProfile() {

        System.out.println("Generating State Profile...");

        double[][] stateProfile_ = new double[3][velocityProfile[0].length];

        double time = 0;

        stateProfile_[0][0] = time;
        stateProfile_[1][0] = 0.0;
        stateProfile_[2][0] = 0.0;

        for(int i = 1; i < stateProfile_[0].length; i++) {

            double avgVel = (velocityProfile[1][i - 1] + velocityProfile[1][i])/2;
            double dt = ds/avgVel;
            time += dt;

            stateProfile_[0][i] = time;

            double K = curvature(paramToArc[0][i]);

            double leftDilation = 1;
            double rightDilation = 1;

            double dir = Math.signum(K);
            K = Math.abs(K);

            if(dir != 0) {
                leftDilation = 1/(1 + K * Context.ROBOT_WIDTH/2);
                rightDilation = 1/(1 - K * Context.ROBOT_WIDTH/2);
            }

            if(dir == -1) {
                double temp = leftDilation;
                leftDilation = rightDilation;
                rightDilation = temp;
            }

            stateProfile_[1][i] = leftDilation * velocityProfile[1][i];

            stateProfile_[2][i] = rightDilation * velocityProfile[1][i];

        }

        System.out.println("Completed State Profile");

        stateProfile = stateProfile_;
    }

    public double curvature(double u) {

        SimpleMatrix velMat = getVelocity(u);
        SimpleMatrix accMat = getAcceleration(u);

        // A Cross Product for 2D can be approximated by making the vectors 3D with 0 as
        // their z value.
        // [ i j k ]
        // [ x1 y1 0 ]
        // [ x2 y2 0 ]
        // magnitude = x1 * y2 - x2 * y1

        // Curvature = ||P''(t) X P'(t)||/(||P'(t)||^3)

        return crossMag(accMat, velMat) / Math.pow(vectorMag(velMat), 3);
    }

    public double crossMag(SimpleMatrix A, SimpleMatrix B) {
        double result = (A.get(0, 0) * B.get(1, 0) - A.get(1, 0) * B.get(0, 0));
        return result;
    }

    public double vectorMag(SimpleMatrix A) {
        return Math.sqrt(A.transpose().mult(A).get(0, 0));
    }

    public SimpleMatrix tMat(double u) {
        double[][] timeMatrix = new double[][] { { u * u * u, u * u, u, 1 } };
        return new SimpleMatrix(timeMatrix);
    }

    public SimpleMatrix getPosition(double u) {
        return new SimpleMatrix(new double[][] { { calcXDeriv(u, 0) }, { calcYDeriv(u, 0) } });
    }

    public SimpleMatrix getVelocity(double u) {
        return new SimpleMatrix(new double[][] { { calcXDeriv(u, 1) }, { calcYDeriv(u, 1) } });
    }

    public SimpleMatrix getAcceleration(double u) {
        return new SimpleMatrix(new double[][] { { calcXDeriv(u, 2) }, { calcYDeriv(u, 2) } });
    }

    public double calcXDeriv(double u, int degree) {

        int splineIndex = (int) (u);
        double splineTime = u - splineIndex;

        SimpleMatrix derivative = cubSplMat[0][splineIndex];

        for (int i = 0; i < degree; i++) {
            derivative = splDerMat.mult(derivative);
        }

        return tMat(splineTime).mult(derivative).get(0, 0);
    }

    public double calcYDeriv(double u, int degree) {

        int splineIndex = (int)(u);
        double splineTime = u - splineIndex;

        SimpleMatrix derivative = cubSplMat[1][splineIndex];

        for (int i = 0; i < degree; i++) {
            derivative = splDerMat.mult(derivative);
        }

        return tMat(splineTime).mult(derivative).get(0, 0);
    }

    public double[] doubleListToArray(ArrayList<Double> list) {

        double[] array = new double[list.size()];

        for (int i = 0; i < list.size(); i++) {
            array[i] = list.get(i);
        }

        return array;
    }

    public SimpleMatrix[][] getCubicSplineMatrix() {
        return cubSplMat;
    }

    public double[][] getParamToArc() {
        return paramToArc;
    }

    public double[][] getVelocityProfile() {
        return velocityProfile;
    }

    public double[][] getStateProfile() {
        return stateProfile;
    }

}