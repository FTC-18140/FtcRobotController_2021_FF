/* -*- mode: java; c-basic-offset: 2; indent-tabs-mode: nil -*- */

/*
  Part of the Processing project - http://processing.org

  Copyright (c) 2012-17 The Processing Foundation
  Copyright (c) 2008-12 Ben Fry and Casey Reas
  Copyright (c) 2008 Dan Shiffman

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
 */

package org.firstinspires.ftc.teamcode.Summer;

import java.io.Serializable;

/**
 * Based on the PVector class found in the Processing language, a class to
 * describe a two or three dimensional vector. This datatype
 * stores two or three variables that are commonly used as a position,
 * velocity, and/or acceleration. Technically, <em>position</em> is a point
 * and <em>velocity</em> and <em>acceleration</em> are vectors, but this is
 * often simplified to consider all three as vectors.
 *
 * For example, if you consider a rectangle moving across the screen, at any given instant it
 * has a position (the object's location, expressed as a point.), a
 * velocity (the rate at which the object's position changes per time unit,
 * expressed as a vector), and acceleration (the rate at which the object's
 * velocity changes per time unit, expressed as a vector). Since vectors
 * represent groupings of values, we cannot simply use traditional
 * addition/multiplication/etc. Instead, we'll need to do some "vector"
 * math, which is made easy by the methods inside the <b>PVector</b>
 * class.<br />
 * <br />
 *
 * The result of all functions are applied to the vector itself, with the
 * exception of cross(), which returns a new PVector (or writes to a specified
 * 'target' PVector). That is, add() will add the contents of one vector to
 * this one. Using add() with additional parameters allows you to put the
 * result into a new PVector. Functions that act on multiple vectors also
 * include static versions. Because creating new objects can be computationally
 * expensive, most functions include an optional 'target' PVector, so that a
 * new PVector object is not created with each operation.
 * <p>
 *
 * @see <a href="https://processing.org">Processing reference</a>
 * @see <a href="https://processing.org/reference/PVector.html">Original PVector class</a>
 */
public class PVector implements Serializable
{

    /**
     * The x component of the vector. This field (variable) can be used to both
     * get and set the value (see above example.)
     */
    public float x;

    /**
     * The y component of the vector. This field (variable) can be used to both
     * get and set the value (see above example.)
     */
    public float y;

    /**
     * The z component of the vector. This field (variable) can be used to both
     * get and set the value (see above example.)
     */
    public float z;

    /** Array so that this can be temporarily used in an array context */
    transient protected float[] array;


    /**
     * Constructor for an empty vector: x, y, and z are set to 0.
     */
    public PVector()
    {

    }

    /**
     * Constructor for a 3D vector.
     *
     * @param  x the x coordinate.
     * @param  y the y coordinate.
     * @param  z the z coordinate.
     */
    public PVector(float x, float y, float z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }


    /**
     * Constructor for a 2D vector: z coordinate is set to 0.
     */
    public PVector(float x, float y)
    {
        this.x = x;
        this.y = y;
    }


    /**
     * Sets the x, y, and z component of the vector using two or three separate
     * variables, the data from a PVector, or the values from a float array.
     *
     * @param x the x component of the vector
     * @param y the y component of the vector
     * @param z the z component of the vector
     */
    public PVector set(float x, float y, float z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }


    /**
     * @param x the x component of the vector
     * @param y the y component of the vector
     */
    public PVector set(float x, float y)
    {
        this.x = x;
        this.y = y;
        this.z = 0;
        return this;
    }


    /**
     * @param v any variable of type PVector
     */
    public PVector set(PVector v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
        return this;
    }


    /**
     * Set the x, y (and maybe z) coordinates using a float[] array as the source.
     * @param source array to copy from
     */
    public PVector set(float[] source)
    {
        if (source.length >= 2)
        {
            x = source[0];
            y = source[1];
        }
        if (source.length >= 3)
        {
            z = source[2];
        }
        else
        {
            z = 0;
        }
        return this;
    }


    /**
     * Make a new 2D unit vector from an angle.
     *
     * @param angle the angle in radians
     * @return the new unit PVector
     */
    static public PVector fromAngle(float angle)
    {
        return fromAngle(angle,null);
    }


    /**
     * Make a new 2D unit vector from an angle
     *
     * @param target the target vector (if null, a new vector will be created)
     * @return the PVector
     */
    static public PVector fromAngle(float angle, PVector target)
    {
        if (target == null)
        {
            target = new PVector((float)Math.cos(angle),(float)Math.sin(angle),0);
        }
        else
        {
            target.set((float)Math.cos(angle),(float)Math.sin(angle),0);
        }
        return target;
    }


    /**
     * Gets a copy of the vector, returns a PVector object.
     */
    public PVector copy()
    {
        return new PVector(x, y, z);
    }


    /**
     * @param target
     */
    public float[] get(float[] target)
    {
        if (target == null)
        {
            return new float[] { x, y, z };
        }
        if (target.length >= 2)
        {
            target[0] = x;
            target[1] = y;
        }
        if (target.length >= 3)
        {
            target[2] = z;
        }
        return target;
    }


    /**
     * Calculates the magnitude (length) of the vector and returns the result
     * as a float (this is simply the equation <em>sqrt(x*x + y*y + z*z)</em>.)
     *
     * @return magnitude (length) of the vector
     * @see PVector#magSq()
     */
    public float mag()
    {
        return (float) Math.sqrt(x*x + y*y + z*z);
    }


    /**
     * Calculates the squared magnitude of the vector and returns the result
     * as a float (this is simply the equation <em>(x*x + y*y + z*z)</em>.)
     * Faster if the real length is not required in the
     * case of comparing vectors, etc.
     * @return squared magnitude of the vector
     * @see PVector#mag()
     */
    public float magSq()
    {
        return (x*x + y*y + z*z);
    }


    /**
     * Adds x, y, and z components to a vector, adds one vector to another, or
     * adds two independent vectors together. The version of the method that
     * adds two vectors together is a static method and returns a PVector, the
     * others have no return value -- they act directly on the vector. See the
     * examples for more context.
     * @param v the vector to be added
     */
    public PVector add(PVector v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return this;
    }


    /**
     * @param x x component of the vector
     * @param y y component of the vector
     */
    public PVector add(float x, float y)
    {
        this.x += x;
        this.y += y;
        return this;
    }


    /**
     * @param z z component of the vector
     */
    public PVector add(float x, float y, float z)
    {
        this.x += x;
        this.y += y;
        this.z += z;
        return this;
    }


    /**
     * Add two vectors
     * @param v1 a vector
     * @param v2 another vector
     */
    static public PVector add(PVector v1, PVector v2)
    {
        return add(v1, v2, null);
    }


    /**
     * Add two vectors into a target vector
     * @param target the target vector (if null, a new vector will be created)
     */
    static public PVector add(PVector v1, PVector v2, PVector target)
    {
        if (target == null)
        {
            target = new PVector(v1.x + v2.x,v1.y + v2.y, v1.z + v2.z);
        }
        else
        {
            target.set(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
        }
        return target;
    }


    /**
     * Subtracts x, y, and z components from a vector, subtracts one vector
     * from another, or subtracts two independent vectors. The version of the
     * method that subtracts two vectors is a static method and returns a
     * PVector, the others have no return value -- they act directly on the
     * vector. See the examples for more context.
     * @param v any variable of type PVector
     */
    public PVector sub(PVector v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return this;
    }


    /**
     * @param x the x component of the vector
     * @param y the y component of the vector
     */
    public PVector sub(float x, float y)
    {
        this.x -= x;
        this.y -= y;
        return this;
    }


    /**
     * @param z the z component of the vector
     */
    public PVector sub(float x, float y, float z)
    {
        this.x -= x;
        this.y -= y;
        this.z -= z;
        return this;
    }


    /**
     * Subtract one vector from another
     * @param v1 the x, y, and z components of a PVector object
     * @param v2 the x, y, and z components of a PVector object
     */
    static public PVector sub(PVector v1, PVector v2)
    {
        return sub(v1, v2, null);
    }


    /**
     * Subtract one vector from another and store in another vector
     * @param target PVector in which to store the result
     */
    static public PVector sub(PVector v1, PVector v2, PVector target)
    {
        if (target == null)
        {
            target = new PVector(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        }
        else
        {
            target.set(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        }
        return target;
    }


    /**
     * Multiplies a vector by a scalar or multiplies one vector by another.
     *
     * @param n the number to multiply with the vector
     */
    public PVector mult(double n)
    {
        x *= n;
        y *= n;
        z *= n;
        return this;
    }


    /**
     * @param v the vector to multiply by the scalar
     */
    static public PVector mult(PVector v, float n)
    {
        return mult(v, n, null);
    }


    /**
     * Multiply a vector by a scalar, and write the result into a target PVector.
     * @param target PVector in which to store the result
     */
    static public PVector mult(PVector v, float n, PVector target)
    {
        if (target == null)
        {
            target = new PVector(v.x*n, v.y*n, v.z*n);
        }
        else
        {
            target.set(v.x*n, v.y*n, v.z*n);
        }
        return target;
    }


    /**
     * Divides a vector by a scalar or divides one vector by another.
     * @param n the number by which to divide the vector
     */
    public PVector div(float n)
    {
        x /= n;
        y /= n;
        z /= n;
        return this;
    }


    /**
     * Divide a vector by a scalar and return the result in a new vector.
     * @param v the vector to divide by the scalar
     * @return a new vector that is v1 / n
     */
    static public PVector div(PVector v, float n)
    {
        return div(v, n, null);
    }


    /**
     * Divide a vector by a scalar and store the result in another vector.
     * @param target PVector in which to store the result
     */
    static public PVector div(PVector v, float n, PVector target)
    {
        if (target == null)
        {
            target = new PVector(v.x/n, v.y/n, v.z/n);
        }
        else
        {
            target.set(v.x/n, v.y/n, v.z/n);
        }
        return target;
    }


    /**
     * Calculates the Euclidean distance between two points (considering a
     * point as a vector object).
     * @param v the x, y, and z coordinates of a PVector
     */
    public float dist(PVector v)
    {
        float dx = x - v.x;
        float dy = y - v.y;
        float dz = z - v.z;
        return (float) Math.sqrt(dx*dx + dy*dy + dz*dz);
    }


    /**
     * @param v1 any variable of type PVector
     * @param v2 any variable of type PVector
     * @return the Euclidean distance between v1 and v2
     */
    static public float dist(PVector v1, PVector v2)
    {
        float dx = v1.x - v2.x;
        float dy = v1.y - v2.y;
        float dz = v1.z - v2.z;
        return (float) Math.sqrt(dx*dx + dy*dy + dz*dz);
    }


    /**
     * Calculates the dot product of two vectors.
     * @param v any variable of type PVector
     * @return the dot product
     */
    public float dot(PVector v)
    {
        return x*v.x + y*v.y + z*v.z;
    }


    /**
     * @param x x component of the vector
     * @param y y component of the vector
     * @param z z component of the vector
     */
    public float dot(float x, float y, float z)
    {
        return this.x*x + this.y*y + this.z*z;
    }


    /**
     * @param v1 any variable of type PVector
     * @param v2 any variable of type PVector
     */
    static public float dot(PVector v1, PVector v2)
    {
        return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
    }


    /**
     * Calculates and returns a vector composed of the cross product between
     * two vectors.
     * @param v the vector to calculate the cross product
     */
    public PVector cross(PVector v)
    {
        return cross(v, null);
    }


    /**
     * @param v any variable of type PVector
     * @param target PVector to store the result
     */
    public PVector cross(PVector v, PVector target)
    {
        float crossX = y * v.z - v.y * z;
        float crossY = z * v.x - v.z * x;
        float crossZ = x * v.y - v.x * y;

        if (target == null)
        {
            target = new PVector(crossX, crossY, crossZ);
        }
        else
        {
            target.set(crossX, crossY, crossZ);
        }
        return target;
    }


    /**
     * @param v1 any variable of type PVector
     * @param v2 any variable of type PVector
     * @param target PVector to store the result
     */
    static public PVector cross(PVector v1, PVector v2, PVector target)
    {
        float crossX = v1.y * v2.z - v2.y * v1.z;
        float crossY = v1.z * v2.x - v2.z * v1.x;
        float crossZ = v1.x * v2.y - v2.x * v1.y;

        if (target == null)
        {
            target = new PVector(crossX, crossY, crossZ);
        }
        else
        {
            target.set(crossX, crossY, crossZ);
        }
        return target;
    }


    /**
     * Normalize the vector to length 1 (make it a unit vector).
     */
    public PVector normalize()
    {
        float m = mag();
        if (m != 0 && m != 1)
        {
            div(m);
        }
        return this;
    }


    /**
     * @param target Set to null to create a new vector
     * @return a new vector (if target was null), or target
     */
    public PVector normalize(PVector target)
    {
        if (target == null)
        {
            target = new PVector();
        }
        float m = mag();
        if (m > 0)
        {
            target.set(x/m, y/m, z/m);
        }
        else
        {
            target.set(x, y, z);
        }
        return target;
    }


    /**
     * Limit the magnitude of this vector to the value used for the <b>max</b> parameter.
     * @param max the maximum magnitude for the vector
     */
    public PVector limit(double max)
    {
        if (magSq() > max*max)
        {
            normalize();
            mult(max);
        }
        return this;
    }


    /**
     * Set the magnitude of this vector to the value used for the <b>len</b> parameter.
     * @param len the new length for this vector
     */
    public PVector setMag(double len)
    {
        normalize();
        mult(len);
        return this;
    }


    /**
     * Sets the magnitude of this vector, storing the result in another vector.
     * @param target Set to null to create a new vector
     * @param len the new length for the new vector
     * @return a new vector (if target was null), or target
     */
    public PVector setMag(PVector target, float len)
    {
        target = normalize(target);
        target.mult(len);
        return target;
    }


    /**
     * Calculate the angle of rotation for this vector (only 2D vectors)
     * @return the angle of rotation
     */
    public float heading()
    {
        return (float) Math.atan2(y, x);
    }


    @Deprecated
    public float heading2D()
    {
        return heading();
    }


    /**
     * Rotate the vector by an angle (only 2D vectors), magnitude remains the same
     * @param theta the angle of rotation
     */
    public PVector rotate(float theta)
    {
        float temp = x;
        // Might need to check for rounding errors like with angleBetween function?
        x = x*(float)(Math.cos(theta)) - y*(float)(Math.sin(theta));
        y = temp*(float)(Math.sin(theta)) + y*(float)(Math.cos(theta));
        return this;
    }

    static public final float lerp(float start, float stop, float amt)
    {
        return start + (stop-start) * amt;
    }


    /**
     * Linear interpolate the vector to another vector
     * @param v the vector to lerp to
     * @param amt  The amount of interpolation; some value between 0.0 (old vector) and 1.0 (new vector). 0.1 is very near the old vector; 0.5 is halfway in between.
     */
    public PVector lerp(PVector v, float amt)
    {
        x = lerp(x, v.x, amt);
        y = lerp(y, v.y, amt);
        z = lerp(z, v.z, amt);
        return this;
    }


    /**
     * Linear interpolate between two vectors (returns a new PVector object)
     * @param v1 the vector to start from
     * @param v2 the vector to lerp to
     */
    public static PVector lerp(PVector v1, PVector v2, float amt)
    {
        PVector v = v1.copy();
        v.lerp(v2, amt);
        return v;
    }


    /**
     * Linear interpolate the vector to x,y,z values
     * @param x the x component to lerp to
     * @param y the y component to lerp to
     * @param z the z component to lerp to
     */
    public PVector lerp(float x, float y, float z, float amt)
    {
        this.x = lerp(this.x, x, amt);
        this.y = lerp(this.y, y, amt);
        this.z = lerp(this.z, z, amt);
        return this;
    }


    /**
     * Calculates and returns the angle (in radians) between two vectors.
     * @param v1 the x, y, and z components of a PVector
     * @param v2 the x, y, and z components of a PVector
     */
    static public float angleBetween(PVector v1, PVector v2)
    {

        // We get NaN if we pass in a zero vector which can cause problems
        // Zero seems like a reasonable angle between a (0,0,0) vector and something else
        if (v1.x == 0 && v1.y == 0 && v1.z == 0 ) return 0.0f;
        if (v2.x == 0 && v2.y == 0 && v2.z == 0 ) return 0.0f;

        double dot = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
        double v1mag = Math.sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
        double v2mag = Math.sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
        // This should be a number between -1 and 1, since it's "normalized"
        double amt = dot / (v1mag * v2mag);
        // But if it's not due to rounding error, then we need to fix it
        // http://code.google.com/p/processing/issues/detail?id=340
        // Otherwise if outside the range, acos() will return NaN
        // http://www.cppreference.com/wiki/c/math/acos
        if (amt <= -1)
        {
            return (float)(Math.PI);
        }
        else if (amt >= 1)
        {
            // http://code.google.com/p/processing/issues/detail?id=435
            return 0;
        }
        return (float) Math.acos(amt);
    }


    @Override
    public String toString()
    {
        return "[ " + x + ", " + y + ", " + z + " ]";
    }


    /**
     * Return a representation of this vector as a float array. This is only
     * for temporary use. If used in any other fashion, the contents should be
     * copied by using the <b>PVector.get()</b> method to copy into your own array.
     */
    public float[] array()
    {
        if (array == null)
        {
            array = new float[3];
        }
        array[0] = x;
        array[1] = y;
        array[2] = z;
        return array;
    }


    @Override
    public boolean equals(Object obj)
    {
        if (!(obj instanceof PVector))
        {
            return false;
        }
        final PVector p = (PVector) obj;
        return x == p.x && y == p.y && z == p.z;
    }


    @Override
    public int hashCode()
    {
        int result = 1;
        result = 31 * result + Float.floatToIntBits(x);
        result = 31 * result + Float.floatToIntBits(y);
        result = 31 * result + Float.floatToIntBits(z);
        return result;
    }
}
