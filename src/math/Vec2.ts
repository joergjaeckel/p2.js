/* Copyright (c) 2013, Brandon Jones, Colin MacKenzie IV. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

import {vec2} from "../../index";

/**
 * The vec2 object from glMatrix, with some extensions and some removed methods. See http://glmatrix.net.
 * @class vec2
 */

var Utils = require('../utils/Utils');

/**
 * 3-dimensional vector
 * @example
 *     const v = new Vec2(1, 2, 3)
 *     console.log('x=' + v.x) // x=1
 */
export class Vec2 {
    x: number
    y: number

    static ZERO: Vec2
    static UNIT_X: Vec2
    static UNIT_Y: Vec2

    constructor(x = 0.0, y = 0.0) {
        this.x = x
        this.y = y
    }

    /**
     * Set all components of the vector to zero.
     */
    setZero(): void {
        this.x = this.y = 0
    }

    /**
     * Make a cross product and only return the z component
     * @method crossLength
     * @static
     * @param  {Array} a
     * @param  {Array} b
     * @return {Number}
     */
    crossLength(a, b) {
        return a[0] * b[1] - a[1] * b[0];
    };

    /**
     * Cross product between a vector and the Z component of a vector
     * @method crossVZ
     * @static
     * @param  {Array} out
     * @param  {Array} vec
     * @param  {Number} zcomp
     * @return {Array}
     */
    crossVZ(out, vec, zcomp) {
        vec.rotate(out, -Math.PI / 2);// Rotate according to the right hand rule
        out.scale(zcomp, out);      // Scale with z
        return out;
    };

    /**
     * Cross product between a vector and the Z component of a vector
     * @method crossZV
     * @static
     * @param  {Array} out
     * @param  {Number} zcomp
     * @param  {Array} vec
     * @return {Array}
     */
    crossZV(out, zcomp, vec) {
        vec.rotate(out, Math.PI / 2); // Rotate according to the right hand rule
        out.scale(zcomp, out);      // Scale with z
        return out;
    };

    /**
     * Vector rotation
     * @param target Optional target to save in.
     */
    rotate(angle: number): Vec2
    rotate(angle: number, target: Vec2): void
    rotate(angle: number, target?: Vec2): Vec2 | void {

        if (angle !== 0) {
            var c = Math.cos(angle),
                s = Math.sin(angle),
                x = this.x,
                y = this.y;

            if (target) {
                target.x = c * x - s * y;
                target.y = s * x + c * y;
            } else {
                return new Vec2(c * x - s * y, s * x + c * y)
            }

            return target;

        } else {

            if (target) {
                target.x = this.x
                target.y = this.y
            } else {
                return new Vec2(this.x, this.y)
            }

            return target;

        }

    }

    /**
     * Rotate a vector 90 degrees clockwise
     * @method rotate90cw
     * @static
     * @param  {Array} out
     * @param  {Array} a
     * @param  {Number} angle
     * @return {Array}
     */
    rotate90cw(out, a) {
        var x = a[0];
        var y = a[1];
        out[0] = y;
        out[1] = -x;
        return out;
    };

    /**
     * Transform a point position to local frame.
     * @method toLocalFrame
     * @param  {Array} out
     * @param  {Array} worldPoint
     * @param  {Array} framePosition
     * @param  {Number} frameAngle
     * @return {Array}
     */
    toLocalFrame(out, worldPoint, framePosition, frameAngle) {
        var c = Math.cos(-frameAngle),
            s = Math.sin(-frameAngle),
            x = worldPoint[0] - framePosition[0],
            y = worldPoint[1] - framePosition[1];
        out[0] = c * x - s * y;
        out[1] = s * x + c * y;
        return out;
    };

    /**
     * Transform a point position to global frame.
     * @method toGlobalFrame
     * @param  {Array} out
     * @param  {Array} localPoint
     * @param  {Array} framePosition
     * @param  {Number} frameAngle
     */
    toGlobalFrame(localPoint: Vec2, framePosition: Vec2, frameAngle: number) {
        var c = Math.cos(frameAngle),
            s = Math.sin(frameAngle),
            x = localPoint[0],
            y = localPoint[1],
            addX = framePosition[0],
            addY = framePosition[1];
        this.x = c * x - s * y + addX;
        this.y = s * x + c * y + addY;
    };

    /**
     * Transform a vector to local frame.
     * @method vectorToLocalFrame
     * @param  {Array} out
     * @param  {Array} worldVector
     * @param  {Number} frameAngle
     * @return {Array}
     */
    vectorToLocalFrame(worldVector: Vec2, frameAngle: number) {
        var c = Math.cos(-frameAngle),
            s = Math.sin(-frameAngle),
            x = worldVector[0],
            y = worldVector[1];
        this.x = c * x - s * y;
        this.y = s * x + c * y;
    };

    /**
     * Transform a vector to global frame.
     * @method vectorToGlobalFrame
     * @param  {Array} out
     * @param  {Array} localVector
     * @param  {Number} frameAngle
     */
    vectorToGlobalFrame() {
        return this.rotate;
    }

    /**
     * Compute centroid of a triangle spanned by vectors a,b,c. See http://easycalculation.com/analytical/learn-centroid.php
     * @method centroid
     * @static
     * @param  {Array} out
     * @param  {Array} a
     * @param  {Array} b
     * @param  {Array} c
     * @return  {Array} The "out" vector.
     */
    centroid(out, a, b, c) {
        a.add(b, out);
        out.add(c, out);
        out.scale(1 / 3, out);
        return out;
    };

    /**
     * Creates a new, empty vec2
     * @static
     * @method create
     * @return {Array} a new 2D vector
     */
    create() {
        var out = new Utils.ARRAY_TYPE(2);
        out[0] = 0;
        out[1] = 0;
        return out;
    };

    /**
     * Clone the vector
     */
    clone(): Vec2 {
        return new Vec2(this.x, this.y)
    };

    /**
     * Creates a new vec2 initialized with the given values
     * @static
     * @method fromValues
     * @param {Number} x X component
     * @param {Number} y Y component
     * @return {Array} a new 2D vector
     */
    fromValues(x, y) {
        var out = new Utils.ARRAY_TYPE(2);
        out[0] = x;
        out[1] = y;
        return out;
    };

    /**
     * Copy the values from one vec2 to another
     */
    copy(vector: Vec2): Vec2{
        this.x = vector.x
        this.y = vector.y
        return this
    };

    /**
     * Set the vectors' 2 elements
     */
    set(x: number, y: number): Vec2 {
        this.x = x
        this.y = y
        return this
    }

    /**
     * Vector addition
     * @param target Optional target to save in.
     */
    add(vector: Vec2): Vec2
    add(vector: Vec2, target: Vec2): void
    add(vector: Vec2, target?: Vec2): Vec2 | void {
        if (target) {
            target.x = this.x + vector.x
            target.y = this.y + vector.y
        } else {
            return new Vec2(this.x + vector.x, this.y + vector.y)
        }
    }

    /**
     * Vector subtraction
     * @param target Optional target to save in.
     */
    subtract(vector: Vec2): Vec2
    subtract(vector: Vec2, target: Vec2): void
    subtract(vector: Vec2, target?: Vec2): Vec2 | void {
        if (target) {
            target.x = this.x - vector.x
            target.y = this.y - vector.y
        } else {
            return new Vec2(this.x - vector.x, this.y - vector.y)
        }
    }

    /**
     * Multiplies two vec2's
     * @static
     * @method multiply
     * @param vector Operand.
     * @param target Optional target to save in.
     */

    multiply(vector: Vec2): Vec2
    multiply(vector: Vec2, target: Vec2): void
    multiply(vector: Vec2, target?: Vec2): Vec2 | void {
        if (target) {
            target.x = this.x * vector.x
            target.y = this.y * vector.y
        } else {
            return new Vec2(this.x * vector.x, this.y * vector.y)
        }
    }

    /**
     * Divides two vec2's
     * @static
     * @method divide
     * @param {Array} out the receiving vector
     * @param {Array} a the first operand
     * @param {Array} b the second operand
     * @return {Array} out
     */
    divide(out, a, b) {
        out[0] = a[0] / b[0];
        out[1] = a[1] / b[1];
        return out;
    };
    /**
     * Multiply all the components of the vector with a scalar.
     * @param target The vector to save the result in.
     */
    scale(scalar: number, target = new Vec2()): Vec2 {
        const x = this.x
        const y = this.y
        target.x = scalar * x
        target.y = scalar * y
        return target
    }

    /**
     * Calculates the euclidian distance between two vec2's
     */
    distanceTo(p: Vec2) {
        var x = p.x[0] - this.x[0],
            y = p.y[1] - this.y[1];
        return Math.sqrt(x * x + y * y);
    };

    /**
     * Get squared distance from this point to another point
     */
    squaredDistance(p: Vec2): number {
        const x = this.x
        const y = this.y
        return x * x + y * y;
    }

    /**
     * Get the length of the vector
     */
    length(): number {
        const x = this.x
        const y = this.y
        return Math.sqrt(x * x + y * y)
    }

    /**
     * Calculates the squared length of a vec2
     * @static
     * @method squaredLength
     * @param {Array} a vector to calculate squared length of
     * @return {Number} squared length of a
     */
    squaredLength() {
        const x = this.x;
        const y = this.y;
        return x * x + y * y;
    };

    /**
     * Negates the components of a vec2
     * @static
     * @method negate
     * @param {Array} out the receiving vector
     * @param {Array} a vector to negate
     * @return {Array} out
     */
    negate(out, a) {
        out[0] = -a[0];
        out[1] = -a[1];
        return out;
    };

    /**
     * Normalize the vector. Note that this changes the values in the vector.
     * @return Returns the norm of the vector
     */

    normalize(target: Vec2): Vec2 {
        const x = this.x;
        const y = this.y;
        let len = x*x + y*y;
        if (len > 0) {
            //TODO: evaluate use of glm_invsqrt here?
            len = 1 / Math.sqrt(len);
            target.x = this.x * len;
            target.y = this.y * len;
        }
        return target;
    }

    /**
     * Calculate dot product
     * @param vector
     */
    dot(vector: Vec2): number {
        return this.x * vector.x + this.y * vector.y
    }

    /**
     * Returns a string representation of a vector
     * @static
     * @method str
     * @param {Array} vec vector to represent as a string
     * @return {String} string representation of the vector
     */
    str(a) {
        return 'vec2(' + a[0] + ', ' + a[1] + ')';
    };

    /**
     * Do a linear interpolation between two vectors
     * @param t A number between 0 and 1. 0 will make this function return u, and 1 will make it return v. Numbers in between will generate a vector in between them.
     */
    lerp(vector: Vec2, t: number, target: Vec2): void {
        const x = this.x
        const y = this.y
        target.x = x + (vector.x - x) * t
        target.y = y + (vector.y - y) * t
    }

    /**
     * Reflect a vector along a normal.
     * @static
     * @method reflect
     * @param {Array} out
     * @param {Array} vector
     * @param {Array} normal
     */
    reflect(out, vector, normal) {
        var dot = vector[0] * normal[0] + vector[1] * normal[1];
        out[0] = vector[0] - 2 * normal[0] * dot;
        out[1] = vector[1] - 2 * normal[1] * dot;
    };

    /**
     * Get the intersection point between two line segments.
     * @static
     * @method getLineSegmentsIntersection
     * @param  {Array} out
     * @param  {Array} p0
     * @param  {Array} p1
     * @param  {Array} p2
     * @param  {Array} p3
     * @return {boolean} True if there was an intersection, otherwise false.
     */
    getLineSegmentsIntersection(out, p0, p1, p2, p3) {
        var t = this.getLineSegmentsIntersectionFraction(p0, p1, p2, p3);
        if (t < 0) {
            return false;
        } else {
            out[0] = p0[0] + (t * (p1[0] - p0[0]));
            out[1] = p0[1] + (t * (p1[1] - p0[1]));
            return true;
        }
    };

    /**
     * Get the intersection fraction between two line segments. If successful, the intersection is at p0 + t * (p1 - p0)
     * @static
     * @method getLineSegmentsIntersectionFraction
     * @param  {Array} p0
     * @param  {Array} p1
     * @param  {Array} p2
     * @param  {Array} p3
     * @return {number} A number between 0 and 1 if there was an intersection, otherwise -1.
     */
    getLineSegmentsIntersectionFraction(p0, p1, p2, p3) {
        var s1_x = p1[0] - p0[0];
        var s1_y = p1[1] - p0[1];
        var s2_x = p3[0] - p2[0];
        var s2_y = p3[1] - p2[1];

        var s, t;
        s = (-s1_y * (p0[0] - p2[0]) + s1_x * (p0[1] - p2[1])) / (-s2_x * s1_y + s1_x * s2_y);
        t = (s2_x * (p0[1] - p2[1]) - s2_y * (p0[0] - p2[0])) / (-s2_x * s1_y + s1_x * s2_y);
        if (s >= 0 && s <= 1 && t >= 0 && t <= 1) { // Collision detected
            return t;
        }
        return -1; // No collision
    };
    
}

Vec2.ZERO = new Vec2(0, 0)
Vec2.UNIT_X = new Vec2(1, 0)
Vec2.UNIT_Y = new Vec2(0, 1)
