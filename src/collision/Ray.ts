import { RaycastResult } from '../collision/RaycastResult'
import { Vec2 } from '../math/Vec2'
import { Shape } from '../shapes/Shape'

/**
 * RAY_MODES
 */
export const RAY_MODES = {
    /** CLOSEST */
    CLOSEST: 1,
    /** ANY */
    ANY: 2,
    /** ALL */
    ALL: 4,
} as const

/**
 * RayMode
 */
export type RayMode = typeof RAY_MODES[keyof typeof RAY_MODES]

export type RayOptions = {
    /**
     * from
     */
    from?: Vec2
    /**
     * to
     */
    to?: Vec2
    /**
     * mode
     */
    mode?: RayMode
    /**
     * If set to `true`, the ray skips any hits with normal.dot(rayDirection) < 0.
     * @default false
     */
    skipBackfaces?: boolean
    /**
     * collisionFilterMask
     * @default -1
     */
    collisionMask?: number
    /**
     * collisionFilterGroup
     * @default -1
     */
    collisionGroup?: number
    /**
     * Set to `false` if you don't want the Ray to take `collisionResponse` flags into account on bodies and shapes.
     * @default true
     */
    checkCollisionResponse?: boolean
    /**
     * callback
     */
    callback?: RaycastCallback
}

export type RaycastCallback = (result: RaycastResult) => void
// leave this here not sure where to place
const intersectBody_worldPosition = new Vec2();

/**
 * A line with a start and end point that is used to intersect shapes. For an example, see {{#crossLink "World/raycast:method"}}World.raycast{{/crossLink}}
 */
export class Ray {
    /**
     * Ray start point.
     */
    from: Vec2

    /**
     * Ray end point
     */
    to: Vec2

    /**
     * Set to true if you want the Ray to take .collisionResponse flags into account on bodies and shapes.
     * @default true
     */
    checkCollisionResponse: boolean

    /**
     * If set to true, the ray skips any hits with normal.dot(rayDirection) < 0.
     * @default false
     */
    skipBackfaces: boolean

    /**
     * @property {number} collisionMask
     * @default -1
     */
    collisionMask: number

    /**
     * @property {number} collisionGroup
     * @default -1
     */
    collisionGroup: number

    /**
     * The intersection mode. Should be {{#crossLink "Ray/ANY:property"}}Ray.ANY{{/crossLink}}, {{#crossLink "Ray/ALL:property"}}Ray.ALL{{/crossLink}} or {{#crossLink "Ray/CLOSEST:property"}}Ray.CLOSEST{{/crossLink}}.
     * @default RAY.ANY
     */
    mode: number

    /**
     * Current, user-provided result callback. Will be used if mode is Ray.ALL.
     */
    callback: RaycastCallback

    /**
     * direction
     */
    direction: Vec2

    /**
     * Length of the ray
     * @readOnly
     * @property {number} length
     */
    length: number

    _currentBody: Body

    _currentShape: Shape

    /**
     * CLOSEST
     */
    static CLOSEST = RAY_MODES.CLOSEST
    /**
     * ANY
     */
    static ANY = RAY_MODES.ANY
    /**
     * ALL
     */
    static ALL = RAY_MODES.ALL

    constructor(from = new Vec2(), to = new Vec2()) {
        this.from = from.clone()
        this.to = to.clone()
        this.direction = new Vec2()
        this.checkCollisionResponse = true
        this.skipBackfaces = false
        this.collisionMask = -1
        this.collisionGroup = -1
        this.mode = Ray.ANY
        this.callback = (result) => {}
        this._currentBody = null;
        this._currentShape = null;
    }

    /**
     * Should be called if you change the from or to point.
     */
    update(){

        // Update .direction and .length
        var d = this.direction;
        this.to.subtract(this.from, d);
        this.length = d.length();
        d.normalize();

    };

    /**
     * @method intersectBodies
     */
    intersectBodies(result, bodies) {
        for (var i = 0, l = bodies.length; !result.shouldStop(this) && i < l; i++) {
            var body = bodies[i];
            var aabb = body.getAABB();
            if(aabb.overlapsRay(this) >= 0 || aabb.containsPoint(this.from)){
                this.intersectBody(result, body);
            }
        }
    };




    /**
     * Shoot a ray at a body, get back information about the hit.
     */
    intersectBody(result, body) {
        var checkCollisionResponse = this.checkCollisionResponse;

        if(checkCollisionResponse && !body.collisionResponse){
            return;
        }

        var worldPosition = intersectBody_worldPosition;

        for (var i = 0, N = body.shapes.length; i < N; i++) {
            var shape = body.shapes[i];

            if(checkCollisionResponse && !shape.collisionResponse){
                continue; // Skip
            }

            if((this.collisionGroup & shape.collisionMask) === 0 || (shape.collisionGroup & this.collisionMask) === 0){
                continue;
            }

            // Get world angle and position of the shape
            shape.position.rotate(worldPosition, body.angle);
            worldPosition.add(body.position);
            var worldAngle = shape.angle + body.angle;

            this.intersectShape(
                result,
                shape,
                worldAngle,
                worldPosition,
                body
            );

            if(result.shouldStop(this)){
                break;
            }
        }
    };


    /**
     * @method intersectShape
     */
    intersectShape(result, shape, angle, position, body) {
        var from = this.from;

        // Checking radius
        var distance = distanceFromIntersectionSquared(from, this.direction, position);
        if (distance > shape.boundingRadius * shape.boundingRadius) {
            return;
        }

        this._currentBody = body;
        this._currentShape = shape;

        shape.raycast(result, this, position, angle);

        this._currentBody = this._currentShape = null;
    };


    /**
     * Get the AABB of the ray.
     */
    getAABB(result) {
        var to = this.to;
        var from = this.from;
        result.lowerBound.set(
            Math.min(to[0], from[0]),
            Math.min(to[1], from[1])
        );
        result.upperBound.set(
            Math.max(to[0], from[0]),
            Math.max(to[1], from[1])
        );
    };


    /**
     * reportIntersection
     */
    reportIntersection(result, fraction, normal, faceIndex) {
        var shape = this._currentShape;
        var body = this._currentBody;

        // Skip back faces?
        if(this.skipBackfaces && normal.dot(this.direction) > 0) {
            return;
        }

        switch(this.mode){

            case Ray.ALL:
                result.set(
                    normal,
                    shape,
                    body,
                    fraction,
                    faceIndex
                );
                this.callback(result);
                break;

            case Ray.CLOSEST:

                // Store if closer than current closest
                if(fraction < result.fraction || !result.hasHit()){
                    result.set(
                        normal,
                        shape,
                        body,
                        fraction,
                        faceIndex
                    );
                }
                break;

            case Ray.ANY:

                // Report and stop.
                result.set(
                    normal,
                    shape,
                    body,
                    fraction,
                    faceIndex
                );
                break;
        }
    };

}


const v0 = new Vec2;

const intersect = new Vec2();

function distanceFromIntersectionSquared(from, direction, position) {

    // v0 is vector from from to position
    position.subtract(from, v0);
    var dot = v0.dot(direction);

    // intersect = direction * dot + from
    direction.scale(dot, intersect);
    intersect.add(from, intersect);

    return position.squaredDistance(intersect);
}

