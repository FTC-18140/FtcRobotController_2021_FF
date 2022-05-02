package org.firstinspires.ftc.teamcode.Summer;

import java.util.ArrayList;

/**
 * Holds the array of x,y points which define the path the robot should drive.
 */
public class Path
{

    // A Path is an arraylist of points (PVector objects)
    private ArrayList<PVector> points;
    // A path has a radius, i.e how far is it ok for the boid to wander off
    float radius;

    Path()
    {
        // Arbitrary radius of 10
        radius = 10;
        points = new ArrayList<PVector>();
    }

    // Add a point to the path
    void addPoint(float x, float y)
    {
        PVector point = new PVector(x, y);
        points.add(point);
    }

    PVector get( int currentSegment )
    {
        return points.get(currentSegment);
    }

    PVector getStart()
    {
        return points.get(0);
    }

    PVector getEnd()
    {
        return points.get(points.size()-1);
    }

    int getNumPoints()
    {
        return points.size();
    }

}