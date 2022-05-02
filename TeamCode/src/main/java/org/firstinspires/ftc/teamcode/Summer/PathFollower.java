package org.firstinspires.ftc.teamcode.Summer;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PathFollower
{

    PVector location;
    PVector velocity;
    PVector acceleration;
    float radius;
    float maxAccel;    // Maximum steering acceleration
    float maxSpeed;    // Maximum speed
    float pathLookahead = 10;

    int currentSegment = 0;
    boolean lastSegment = false;
    boolean arrived = false;

    PVector start = null;
    PVector end = null;
    PVector pursuitVelocity = null;


    Telemetry telemetry;

    /**
     * Constructor to create a PathFollower
     * @param theInitLocation the initial location
     * @param theMaxSpeed the maximum speed for the robot
     * @param theMaxAccel the maximum acceleration for the robot
     * @param theTelemetry the telemetry object needed to report status
     */
    PathFollower(PVector theInitLocation, float theMaxSpeed, float theMaxAccel, Telemetry theTelemetry)
    {
        location = theInitLocation.copy();
        radius = 4.0f;
        maxSpeed = theMaxSpeed;
        maxAccel = theMaxAccel;
        acceleration = new PVector(0, 0);
        velocity = new PVector(maxSpeed, 0);
        telemetry = theTelemetry;
    }


    /**
     * This function implements Craig Reynolds' path following algorithm to follow the Path parameter
     * http://www.red3d.com/cwr/steer/PathFollow.html
     * @param drivePath the Path object used to define where the robot should drive
     * @return the current steering velocity needed by the robot to follow the specified path
     */
    public PVector follow(Path drivePath)
    {

        PVector target;
        start = drivePath.get(currentSegment);
        end = drivePath.get(currentSegment + 1);
        lastSegment = false;

        //
        //  Determine if I am targeting the last point, then set a flag which indicates I am on the
        //  last segment.  This has implications for when I get to the end.... I need to slow down
        //  and stop, since it is the last point.
        //
        if (drivePath.getNumPoints() == currentSegment + 2)
        {
            lastSegment = true;
        }
        telemetry.addData("CurrentSegment, Last Segment?: ", "%d, %s", currentSegment, String.valueOf(lastSegment));


        //
        //  1. Calculate a Projected Future Location
        //
        //  Do this by taking the current location and current velocity and calculating a
        //  projected location a few inches out from the current location in the direction the current
        //  velocity is pointing.
        //
        PVector velocityCopy = velocity.copy();
        velocityCopy.setMag(pathLookahead);

        PVector projectedLoc = PVector.add(location, velocityCopy);

        //
        //  2. Calculate the Desired Target Location to shoot for
        //
        //   Do this by taking the Projected Location and projecting it onto the current path segment.
        //   This projection is the point on the path segment which is closest to the Projected Location
        //   calculated in the previous step.  This point is called the Normal Point since it represents
        //   the point on the path which is perpendicular to the Projected Location of the robot.
        //
        //   Once this Normal Point has been calculated, shift this point along the path towards the
        //   endpoint to give the robot a nice target ahead to shoot for.
        //
        //   There are a few cases that have to be checked when the Normal Point is close to the
        //   start and end of the path segment.  For example, if the Normal Point is close to the
        //   end point, it's time to switch to the next segment.  Also, if the Normal Point is calculated
        //   to be outside of the segment defined by start and end, it must be corrected.
        //
        if (projectedLoc.dist(end) < radius && lastSegment)
        {
            target = end.copy();
        }
        else
        {
            PVector normalPoint = getNormalPoint(projectedLoc, start, end);

            if (normalPoint.dist(end) > start.dist(end)) {
                normalPoint = start;
            }
            else if (normalPoint.dist(start) > end.dist(start)) {
                normalPoint = end;
            }

            PVector pathDirection = PVector.sub(end, start);
            pathDirection.setMag(pathLookahead);

            target = normalPoint.copy();

            if ( target != end )
            {
                target.add(pathDirection);
            }
        }

        telemetry.addData("Target loc: ", target);

        if (projectedLoc.dist(end) < radius && !lastSegment)
        {
            currentSegment++;
        }

        //
        // 3.  Calculate the Steering Vector to achieve the Target Location
        //
        return arrive(target);
    }


    /**
     * A function to get the normal point from a point (p) to a line segment (a-b)
     * This function could be optimized to make fewer new Vector objects
     * @param p the point which is off the line segment
     * @param a the beginning point of the line segment
     * @param b the end point of the line segment
     */
    private PVector getNormalPoint(PVector p, PVector a, PVector b)
    {
        // Vector from a to p
        PVector ap = PVector.sub(p, a);
        // Vector from a to b
        PVector ab = PVector.sub(b, a);
        ab.normalize(); // Normalize the line
        // Project vector "diff" onto line by using the dot product
        ab.mult(ap.dot(ab));
        PVector normalPoint = PVector.add(a, ab);
        return normalPoint;
    }

    /**
     * Calculate the velocity required to cause the robot to move towards the target location.
     * @param target the point the robot is driving towards
     * @return the needed velocity to steer the robot towards the target
     */
    private PVector arrive( PVector target )
    {
        //
        // Find the needed velocity to move to target and call it desiredVelocity.  This is calculated
        // by subtraction my current vector location from the target vector location.  We care calling
        // this velocity because it represents a change in position we desire to achieve over the
        // next period of time.
        //
        PVector desiredVelocity = PVector.sub(target, location);
        //speed is the magnitude of desiredVelocity
        float speed = desiredVelocity.mag();

        //
        // If the robot is close to the end of the segment, slow down by scaling the desiredVelocity
        // based on the distance to the end.
        //
        if(location.dist(end) < radius)
        {
            float m = (float)Range.scale(speed, 0.0, radius, 0.0, maxSpeed);
            desiredVelocity.setMag(m);
            if( location.dist(end) < 1 )
            {
                arrived = true;
            }
        }
        else
        {
            //set speed to maximum allowed speed
            desiredVelocity.setMag(maxSpeed);
        }

        //
        //  Find the amount of velocity change is needed and call it steerAcceleration.  This is
        //  calculated by taking the desiredVelocity to reach the target location and subtract the
        //  robot's current velocity.  We care calling this acceleration because it represents a
        //  change in velocity we desire to achieve over the next period of time.
        //
        PVector steerAcceleration = PVector.sub(desiredVelocity, velocity);

        // limit rate of change to robot velocity
        steerAcceleration.limit(maxAccel);

        //
        // Corrects robot velocity by adding our steerAcceleration to it.  This results in a new
        // pursuitVelocity that will cause the robot move towards the target location.
        //
        pursuitVelocity = PVector.add(velocity, steerAcceleration);

        //make sure pursuitVelocity isn't too fast
        return pursuitVelocity.limit(maxSpeed);

    }

    /**
     * Reset the follower so that a new Path can be followed.
     */
    public void resetCurrentSegment()
    {
        this.currentSegment = 0;
        this.arrived = false;
    }

    ///////////////////////////////////
    //  Getters and Setters
    ///////////////////////////////////

    /**
     * Updates the robot's current location.
     * @param currentPosition The position being added to the location vector.
     */
    public void setPosition(PVector currentPosition)
    {
        this.location = PVector.add(location, currentPosition);
    }

    /**
     * Updates the robot's velocity.
     * @param currentVelocity The velocity vector being set.
     */
    public void setVelocity(PVector currentVelocity)
    {
        this.velocity.set(currentVelocity.x, currentVelocity.y);
    }

    public float getRadius()
    {
        return radius;
    }

    public void setRadius(float radius)
    {
        this.radius = radius;
    }

    public float getMaxAccel()
    {
        return maxAccel;
    }

    public void setMaxAccel(float maxAccel)
    {
        this.maxAccel = maxAccel;
    }

    public float getMaxSpeed()
    {
        return maxSpeed;
    }

    public void setMaxSpeed(float maxSpeed)
    {
        this.maxSpeed = maxSpeed;
    }

    public float getPathLookahead()
    {
        return pathLookahead;
    }

    public void setPathLookahead(float pathLookahead)
    {
        this.pathLookahead = pathLookahead;
    }

}