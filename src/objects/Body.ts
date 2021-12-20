import { Vec2 } from '../math/Vec2';
import decomp from 'poly-decomp';
import { Convex } from '../shapes/Convex';
import { RaycastResult } from '../collision/RaycastResult';
import { Ray, RAY_MODES } from '../collision/Ray';
import { AABB } from '../collision/AABB';
import { EventEmitter } from '../events/EventEmitter';
import type { World } from '../world/World'
import { Material } from "../material/Material";
import { Shape } from "../shapes/Shape";

/**
 * BODY_TYPES
 */
export const BODY_TYPES = {
    /** DYNAMIC */
    DYNAMIC: 1,
    /** STATIC */
    STATIC: 2,
    /** KINEMATIC */
    KINEMATIC: 4,
} as const

/**
 * BodyType
 */
export type BodyType = typeof BODY_TYPES[keyof typeof BODY_TYPES]

/**
 * BODY_SLEEP_STATES
 */
export const BODY_SLEEP_STATES = {
    /** AWAKE */
    AWAKE: 0,
    /** SLEEPY */
    SLEEPY: 1,
    /** SLEEPING */
    SLEEPING: 2,
} as const

/**
 * BodySleepState
 */
export type BodySleepState = typeof BODY_SLEEP_STATES[keyof typeof BODY_SLEEP_STATES]

export type BodyOptions = ConstructorParameters<typeof Body>[0]

/**
 * Base class for all body types.
 * @example
 *     const shape = new CANNON.Sphere(1)
 *     const body = new CANNON.Body({
 *       mass: 1,
 *       shape,
 *     })
 *     world.addBody(body)
 */

export class Body extends EventEmitter {
    static idCounter = 0

    /**
     * Dispatched after two bodies collide. This event is dispatched on each
     * of the two bodies involved in the collision.
     * @event collide
     * @param body The body that was involved in the collision.
     * @param contact The details of the collision.
     */
    static COLLIDE_EVENT_NAME = 'collide'

    /**
     * A dynamic body is fully simulated. Can be moved manually by the user, but normally they move according to forces. A dynamic body can collide with all body types. A dynamic body always has finite, non-zero mass.
     */
    static DYNAMIC = BODY_TYPES.DYNAMIC

    /**
     * A static body does not move during simulation and behaves as if it has infinite mass. Static bodies can be moved manually by setting the position of the body. The velocity of a static body is always zero. Static bodies do not collide with other static or kinematic bodies.
     */
    static STATIC = BODY_TYPES.STATIC

    /**
     * A kinematic body moves under simulation according to its velocity. They do not respond to forces. They can be moved manually, but normally a kinematic body is moved by setting its velocity. A kinematic body behaves as if it has infinite mass. Kinematic bodies do not collide with other static or kinematic bodies.
     */
    static KINEMATIC = BODY_TYPES.KINEMATIC

    /**
     * AWAKE
     */
    static AWAKE = BODY_SLEEP_STATES.AWAKE
    /**
     * SLEEPY
     */
    static SLEEPY = BODY_SLEEP_STATES.SLEEPY
    /**
     * SLEEPING
     */
    static SLEEPING = BODY_SLEEP_STATES.SLEEPING

    /**
     * Dispatched after a sleeping body has woken up.
     * @event wakeup
     */
    static wakeupEvent = { type: 'wakeup' }

    /**
     * Dispatched after a body has gone in to the sleepy state.
     * @event sleepy
     */
    static sleepyEvent = { type: 'sleepy' }

    /**
     * Dispatched after a body has fallen asleep.
     * @event sleep
     */
    static sleepEvent = { type: 'sleep' }

    /**
     * Identifier of the body.
     */
    id: number

    /**
     * Position of body in World.bodies. Updated by World and used in ArrayCollisionMatrix.
     */
    index: number

    /**
     * Reference to the world the body is living in.
     */
    world: World | null

    /**
     * Callback function that is used BEFORE stepping the system. Use it to apply forces, for example. Inside the function, "this" will refer to this Body object. Deprecated - use World events instead.
     * @deprecated Use World events instead
     */
    preStep: (() => void) | null

    /**
     * Callback function that is used AFTER stepping the system. Inside the function, "this" will refer to this Body object. Deprecated - use World events instead.
     * @deprecated Use World events instead
     */
    postStep: (() => void) | null

    vlambda: Vec2

    /**
     * The collision group the body belongs to.
     * @default 1
     */
    collisionFilterGroup: number

    /**
     * Whether to produce contact forces when in contact with other bodies. Note that contacts will be generated, but they will be disabled - i.e. "collide" events will be raised, but forces will not be altered.
     */
    collisionResponse: boolean

    /**
     * World space position of the body.
     */
    position: Vec2

    previousPosition: Vec2

    /**
     * Interpolated position of the body.
     */
    interpolatedPosition: Vec2

    /**
     * Initial position of the body.
     */
    initPosition: Vec2

    /**
     * World space velocity of the body.
     */
    velocity: Vec2

    /**
     * Linear force on the body in world space.
     */
    force: Vec2

    /**
     * The mass of the body.
     * @default 0
     */
    mass: number

    invMass: number

    /**
     * The physics material of the body. It defines the body interaction with other bodies.
     */
    material: Material | null

    damping: number

    /**
     * One of: `Body.DYNAMIC`, `Body.STATIC` and `Body.KINEMATIC`.
     */
    type: BodyType

    /**
     * If true, the body will automatically fall to sleep.
     * @default true
     */
    allowSleep: boolean

    wantsToSleep: boolean

    /**
     * Current sleep state.
     */
    sleepState: BodySleepState

    /**
     * If the speed (the norm of the velocity) is smaller than this value, the body is considered sleepy.
     * @default 0.1
     */
    sleepSpeedLimit: number

    /**
     * If the body has been sleepy for this sleepTimeLimit seconds, it is considered sleeping.
     * @default 1
     */
    sleepTimeLimit: number

    timeLastSleepy: number

    /**
     * World space rotational force on the body, around center of mass.
     */
    torque: Vec2

    angle: number
    previousAngle: number
    interpolatedAngle: number

    /**
     * Angular velocity of the body, in world space. Think of the angular velocity as a vector, which the body rotates around. The length of this vector determines how fast (in radians per second) the body rotates.
     */
    angularVelocity: number

    angularForce: number

    /**
     * List of Shapes that have been added to the body.
     */
    shapes: Shape[]

    /**
     * Position of each Shape in the body, given in local Body space.
     */
    shapeOffsets: Vec2[]

    /**
     * The inertia of the body.
     */
    inertia: number

    invInertia: number
    invMassSolve: number
    invInertiaSolve: number

    /**
     * Set to true if you don't want the body to rotate. Make sure to run .updateMassProperties() if you change this after the body creation.
     * @default false
     */
    fixedRotation: boolean

    /**
     * How much to damp the body angular velocity each step. It can go from 0 to 1.
     * @default 0.01
     */
    angularDamping: number

    /**
     * World space bounding box of the body and its shapes.
     */
    aabb: AABB

    /**
     * Indicates if the AABB needs to be updated before use.
     */
    aabbNeedsUpdate: boolean

    /**
     * Total bounding radius of the Body including its shapes, relative to body.position.
     */
    boundingRadius: number
    wlambda: number

    fixedX: boolean
    fixedY: boolean

    massMultiplier: Vec2

    gravityScale: number
    idleTime: number
    ccdSpeedThreshold: number
    ccdIterations: number
    islandId: number
    concavePath: []
    _wakeUpAfterNarrowphase: boolean

    constructor(
        options: {
            /**
             * Whether to produce contact forces when in contact with other bodies. Note that contacts will be generated, but they will be disabled - i.e. "collide" events will be raised, but forces will not be altered.
             */
            collisionResponse?: boolean
            /**
             * World space position of the body.
             */
            position?: Vec2
            /**
             * World space velocity of the body.
             */
            velocity?: Vec2
            /**
             * The mass of the body.
             * @default 0
             */
            mass?: number
            /**
             * One of: `Body.DYNAMIC`, `Body.STATIC` and `Body.KINEMATIC`.
             */
            type?: BodyType
            /**
             * If true, the body will automatically fall to sleep.
             * @default true
             */
            allowSleep?: boolean
            /**
             * If the speed (the norm of the velocity) is smaller than this value, the body is considered sleepy.
             * @default 0.1
             */
            sleepSpeedLimit?: number
            /**
             * If the body has been sleepy for this sleepTimeLimit seconds, it is considered sleeping.
             * @default 1
             */
            sleepTimeLimit?: number
            /**
             * World space orientation of the body.
             */
            angle?: number
            /**
             * Angular velocity of the body, in world space. Think of the angular velocity as a vector, which the body rotates around. The length of this vector determines how fast (in radians per second) the body rotates.
             */
            angularVelocity?: number
            /**
             * Set to true if you don't want the body to rotate. Make sure to run .updateMassProperties() if you change this after the body creation.
             * @default false
             */
            fixedRotation?: boolean
            /**
             * How much to damp the body angular velocity each step. It can go from 0 to 1.
             * @default 0.01
             */
            angularDamping?: number

            angularForce?: number

            ccdIterations?: number

            ccdSpeedThreshold?: number

            damping?: number

            fixedX?: boolean

            fixedY?: boolean

            force?: Vec2

            gravityScale?: number

        } = {}
    ) {
        super();

        this.id = Body.idCounter++
        this.index = -1;
        this.world = null;

        this.shapes = [];

        this.mass = options.mass || 0;
        this.invMass = 0;
        this.inertia = 0;
        this.invInertia = 0;
        this.invMassSolve = 0;
        this.invInertiaSolve = 0;

        this.fixedRotation = !!options.fixedRotation;
        this.fixedX = !!options.fixedX;
        this.fixedY = !!options.fixedY;

        this.massMultiplier = new Vec2();

        this.position = options.position ? options.position.clone() : new Vec2();
        this.interpolatedPosition = this.position.clone();
        this.previousPosition = this.position.clone();

        this.velocity = options.velocity ? options.velocity.clone() : new Vec2();

        this.vlambda = new Vec2();

        this.wlambda = 0;

        this.angle = options.angle || 0;
        this.previousAngle = this.angle;
        this.interpolatedAngle = this.angle;
        this.angularVelocity = options.angularVelocity || 0;

        this.force = options.force ? options.force.clone() : new Vec2();
        this.angularForce = options.angularForce || 0;

        this.damping = options.damping !== undefined ? options.damping : 0.1;
        this.angularDamping = options.angularDamping !== undefined ? options.angularDamping : 0.1;

        this.type = this.mass <= 0.0 ? Body.STATIC : Body.DYNAMIC

        if (typeof options.type === typeof Body.STATIC) {
            this.type = options.type!
        }

        this.boundingRadius = 0;

        this.aabb = new AABB();
        this.aabbNeedsUpdate = true;

        this.allowSleep = options.allowSleep !== undefined ? options.allowSleep : true;
        this.wantsToSleep = false;
        this.sleepState = Body.AWAKE;
        this.sleepSpeedLimit = options.sleepSpeedLimit !== undefined ? options.sleepSpeedLimit : 0.2;
        this.sleepTimeLimit = options.sleepTimeLimit !== undefined ? options.sleepTimeLimit : 1;

        this.gravityScale = options.gravityScale !== undefined ? options.gravityScale : 1;
        this.collisionResponse = options.collisionResponse !== undefined ? options.collisionResponse : true;

        this.idleTime = 0;

        this.timeLastSleepy = 0;

        this.ccdSpeedThreshold = options.ccdSpeedThreshold !== undefined ? options.ccdSpeedThreshold : -1;
        this.ccdIterations = options.ccdIterations !== undefined ? options.ccdIterations : 10;

        this.islandId = -1;

        this.concavePath = null;

        this._wakeUpAfterNarrowphase = false;

        this.updateMassProperties();
    }

    /**
     * @private
     * @method updateSolveMassProperties
     */
    updateSolveMassProperties() {
        if(this.sleepState === Body.SLEEPING || this.type === Body.KINEMATIC){
            this.invMassSolve = 0;
            this.invInertiaSolve = 0;
        } else {
            this.invMassSolve = this.invMass;
            this.invInertiaSolve = this.invInertia;
        }
    }

    /**
     * Set the total density of the body
     * @method setDensity
     * @param {number} density
     */
    setDensity(density) {
        const totalArea = this.getArea();
        this.mass = totalArea * density;
        this.updateMassProperties();
    }

    /**
     * Get the total area of all shapes in the body
     * @method getArea
     * @return {Number}
     */
    getArea() {
        let totalArea = 0;
        for(let i=0; i<this.shapes.length; i++){
            totalArea += this.shapes[i].area;
        }
        return totalArea;
    }

    /**
     * Get the AABB from the body. The AABB is updated if necessary.
     * @method getAABB
     * @return {AABB} The AABB instance from the body.
     */
    getAABB() {
        if(this.aabbNeedsUpdate){
            this.updateAABB();
        }
        return this.aabb;
    }

    /**
     * Updates the AABB of the Body, and set .aabbNeedsUpdate = false.
     * @method updateAABB
     */
    updateAABB() {
        const shapes = this.shapes;
        const N = shapes.length;
        const offset = tmp;
        const bodyAngle = this.angle;

        for(let i=0; i!==N; i++){
            const shape = shapes[i];
            const angle = shape.angle + bodyAngle;

            // Get shape world offset
            offset.toGlobalFrame(shape.position, this.position, bodyAngle);

            // Get shape AABB
            shape.computeAABB(shapeAABB, offset, angle);

            if(i===0){
                this.aabb.copy(shapeAABB);
            } else {
                this.aabb.extend(shapeAABB);
            }
        }

        this.aabbNeedsUpdate = false;
    }

    /**
     * Update the bounding radius of the body (this.boundingRadius). Should be done if any of the shape dimensions or positions are changed.
     * @method updateBoundingRadius
     */
    updateBoundingRadius() {
        const shapes = this.shapes;
        const N = shapes.length;
        let radius = 0;

        for(let i=0; i!==N; i++){
            const shape = shapes[i];
            const offset = shape.position.length();
            const r = shape.boundingRadius;
            if(offset + r > radius){
                radius = offset + r;
            }
        }

        this.boundingRadius = radius;
    }

    /**
     * Add a shape to the body. You can pass a local transform when adding a shape,
     * so that the shape gets an offset and angle relative to the body center of mass.
     * Will automatically update the mass properties and bounding radius.
     *
     * @method addShape
     * @param  {Shape}              shape
     * @param  {Array} [offset] Local body offset of the shape.
     * @param  {Number}             [angle]  Local body angle.
     *
     * @example
     *     var body = new Body(),
     *         shape = new Circle({ radius: 1 });
     *
     *     // Add the shape to the body, positioned in the center
     *     body.addShape(shape);
     *
     *     // Add another shape to the body, positioned 1 unit length from the body center of mass along the local x-axis.
     *     body.addShape(shape,[1,0]);
     *
     *     // Add another shape to the body, positioned 1 unit length from the body center of mass along the local y-axis, and rotated 90 degrees CCW.
     *     body.addShape(shape,[0,1],Math.PI/2);
     */
    addShape(shape, offset, angle) {
        if(shape.body){
            throw new Error('A shape can only be added to one body.');
        }
        const world = this.world;
        if(world && world.stepping){
            throw new Error('A shape cannot be added during step.');
        }
        shape.body = this;

        // Copy the offset vector
        if(offset){
            shape.position.copy(offset);
        } else {
            shape.position.set(0, 0);
        }

        shape.angle = angle || 0;

        this.shapes.push(shape);
        this.updateMassProperties();
        this.updateBoundingRadius();

        this.aabbNeedsUpdate = true;
    }

    /**
     * Remove a shape.
     * @method removeShape
     * @param  {Shape} shape
     * @return {Boolean} True if the shape was found and removed, else false.
     */
    removeShape(shape) {
        const world = this.world;
        if(world && world.stepping){
            throw new Error('A shape cannot be removed during step.');
        }

        const idx = this.shapes.indexOf(shape);

        if(idx !== -1){
            this.shapes.splice(idx,1);
            this.aabbNeedsUpdate = true;
            shape.body = null;
            return true;
        } else {
            return false;
        }
    }

    /**
     * Updates .inertia, .invMass, .invInertia for this Body. Should be called when changing the structure or mass of the Body.
     *
     * @method updateMassProperties
     *
     * @example
     *     body.mass += 1;
     *     body.updateMassProperties();
     */
    updateMassProperties() {
        if(this.type === Body.STATIC || this.type === Body.KINEMATIC){

            this.mass = Number.MAX_VALUE;
            this.invMass = 0;
            this.inertia = Number.MAX_VALUE;
            this.invInertia = 0;

        } else {
            const shapes = this.shapes;
            const N = shapes.length;
            let I = 0;

            if(!this.fixedRotation){
                for(let i=0; i<N; i++){
                    const shape = shapes[i];
                    const r2 = shape.position.squaredLength();
                    const Icm = shape.computeMomentOfInertia();
                    I += Icm + r2;
                }
                this.inertia = this.mass * I;
                this.invInertia = I>0 ? 1/I : 0;

            } else {
                this.inertia = Number.MAX_VALUE;
                this.invInertia = 0;
            }

            // Inverse mass properties are easy
            this.invMass = 1 / this.mass;

            this.massMultiplier.set(
                this.fixedX ? 0 : 1,
                this.fixedY ? 0 : 1
            );
        }
    }

    /**
     * Apply force to a point relative to the center of mass of the body. This could for example be a point on the Body surface. Applying force this way will add to Body.force and Body.angularForce.
     * @method applyForce
     * @param  {Array} force The force vector to add, oriented in world space.
     * @param  {Array} [relativePoint] A point relative to the body in world space. If not given, it is set to zero and all of the force will be exerted on the center of mass.
     * @example
     *     var body = new Body({ mass: 1 });
     *     var relativePoint = [1, 0]; // Will apply the force at [body.position[0] + 1, body.position[1]]
     *     var force = [0, 1]; // up
     *     body.applyForce(force, relativePoint);
     *     console.log(body.force); // [0, 1]
     *     console.log(body.angularForce); // 1
     */
    applyForce(force, relativePoint) {

        // Add linear force
        this.force.add(this.force, force);

        if(relativePoint){

            // Compute produced rotational force
            const rotForce = relativePoint.crossLength(force);

            // Add rotational force
            this.angularForce += rotForce;
        }
    }

    applyForceLocal(localForce, localPoint = Body_applyForce_pointLocal) {
        const worldForce = Body_applyForce_forceWorld;
        const worldPoint = Body_applyForce_pointWorld;
        this.vectorToWorldFrame(worldForce, localForce);
        this.vectorToWorldFrame(worldPoint, localPoint);
        this.applyForce(worldForce, worldPoint);
    }

    applyImpulse(impulseVector, relativePoint) {
        if(this.type !== Body.DYNAMIC){
            return;
        }

        // Compute produced central impulse velocity
        const velo = Body_applyImpulse_velo;
        impulseVector.scale(this.invMass, velo);
        velo.multiply(this.massMultiplier);

        // Add linear impulse
        this.velocity.add(velo, this.velocity);

        if(relativePoint){
            // Compute produced rotational impulse velocity
            let rotVelo = relativePoint.crossLength(impulseVector);
            rotVelo *= this.invInertia;

            // Add rotational Impulse
            this.angularVelocity += rotVelo;
        }
    }

    applyImpulseLocal(localImpulse, localPoint = Body_applyImpulse_pointLocal) {
        const worldImpulse = Body_applyImpulse_impulseWorld;
        const worldPoint = Body_applyImpulse_pointWorld;
        this.vectorToWorldFrame(worldImpulse, localImpulse);
        this.vectorToWorldFrame(worldPoint, localPoint);
        this.applyImpulse(worldImpulse, worldPoint);
    }

    /**
     * Transform a world point to local body frame.
     * @method toLocalFrame
     * @param  {Array} out          The point to store the result in
     * @param  {Array} worldPoint   The input world point
     */
    toLocalFrame(out, worldPoint) {
        out.toLocalFrame(worldPoint, this.position, this.angle);
    }

    /**
     * Transform a local point to world frame.
     * @method toWorldFrame
     * @param  {Array} out          The point to store the result in
     * @param  {Array} localPoint   The input local point
     */
    toWorldFrame(out, localPoint) {
        out.toGlobalFrame(localPoint, this.position, this.angle);
    }

    /**
     * Transform a world vector to local body frame.
     * @method vectorToLocalFrame
     * @param  {Array} out          The vector to store the result in
     * @param  {Array} worldVector  The input world vector
     */
    vectorToLocalFrame(out, worldVector) {
        out.vectorToLocalFrame(worldVector, this.angle);
    }

    /**
     * Transform a local vector to world frame.
     * @method vectorToWorldFrame
     * @param  {Array} out          The vector to store the result in
     * @param  {Array} localVector  The input local vector
     */
    vectorToWorldFrame(out, localVector) {
        out.vectorToGlobalFrame(localVector, this.angle);
    }

    /**
     * Reads a polygon shape path, and assembles convex shapes from that and puts them at proper offset points.
     * @method fromPolygon
     * @param {Array} path An array of 2d vectors, e.g. [[0,0],[0,1],...] that resembles a concave or convex polygon. The shape must be simple and without holes.
     * @param {Object} [options]
     * @param {Boolean} [options.optimalDecomp=false]   Set to true if you need optimal decomposition. Warning: very slow for polygons with more than 10 vertices.
     * @param {Boolean} [options.skipSimpleCheck=false] Set to true if you already know that the path is not intersecting itself.
     * @param {Boolean|Number} [options.removeCollinearPoints=false] Set to a number (angle threshold value) to remove collinear points, or false to keep all points.
     * @return {Boolean} True on success, else false.
     * @example
     *     var body = new Body();
     *     var path = [
     *         [-1, 1],
     *         [-1, 0],
     *         [1, 0],
     *         [1, 1],
     *         [0.5, 0.5]
     *     ];
     *     body.fromPolygon(path);
     *     console.log(body.shapes); // [Convex, Convex, ...]
     */
    fromPolygon(path, options: object = {}) {
        // Remove all shapes
        for(let i = this.shapes.length; i >= 0; --i){
            this.removeShape(this.shapes[i]);
        }

        // Copy the path
        const p = [];
        for(let i = 0; i < path.length; i++){
            p[i] = path[i].clone();
        }

        // Make it counter-clockwise
        decomp.makeCCW(p);

        if(options.removeCollinearPoints !== undefined){
            decomp.removeCollinearPoints(p, options.removeCollinearPoints);
        }

        // Check if any line segment intersects the path itself
        if(!options.skipSimpleCheck){
            if(!decomp.isSimple(p)){
                return false;
            }
        }

        // Save this path for later
        const concavePath = this.concavePath = [];
        for(let i = 0; i < p.length; i++){
            concavePath[i] = p[i].clone();
        }

        // Slow or fast decomp?
        let convexes;
        if(options.optimalDecomp){
            convexes = decomp.decomp(p);
        } else {
            convexes = decomp.quickDecomp(p);
        }

        const cm = new Vec2();

        // Add convexes
        for(var i=0; i!==convexes.length; i++){
            // Create convex
            let c = new Convex({ vertices: convexes[i] });

            // Move all vertices so its center of mass is in the local center of the convex
            for(let j=0; j!==c.vertices.length; j++){
                const v = c.vertices[j];
                v.subtract(v,c.centerOfMass);
            }

            cm.copy(c.centerOfMass);

            c = new Convex({ vertices: c.vertices });

            // Add the shape
            this.addShape(c,cm, 0);
        }

        this.adjustCenterOfMass();

        this.aabbNeedsUpdate = true;

        return true;
    }

    /**
     * Moves the shape offsets so their center of mass becomes the body center of mass.
     * @method adjustCenterOfMass
     * @example
     *     var body = new Body({ position: [0, 0] });
     *     var shape = new Circle({ radius: 1 });
     *     body.addShape(shape, [1, 0], 0);
     *     body.adjustCenterOfMass();
     *     console.log(body.position); // [1, 0]
     *     console.log(shape.position); // [0, 0]
     */
    adjustCenterOfMass() {
        const offset_times_area = adjustCenterOfMass_tmp2;
        const sum =               adjustCenterOfMass_tmp3;
        const cm =                adjustCenterOfMass_tmp4;
        let totalArea =         0;
        sum.set(0,0);

        for(let i = 0; i !== this.shapes.length; i++){
            const s = this.shapes[i];
            s.position.scale(s.area, offset_times_area);
            sum.add(sum, offset_times_area);
            totalArea += s.area;
        }

        sum.scale(1/totalArea, cm);

        // Now move all shapes
        for(let i = 0; i !== this.shapes.length; i++){
            const s = this.shapes[i];
            s.position.subtract(cm);
        }

        // Move the body position too
        this.position.add(cm);

        // And concave path
        for(var i=0; this.concavePath && i<this.concavePath.length; i++){
            this.concavePath[i].subtract(cm);
        }

        this.updateMassProperties();
        this.updateBoundingRadius();
    }

    /**
     * Sets the force on the body to zero.
     * @method setZeroForce
     */
    setZeroForce() {
        const f = this.force;
        f[0] = f[1] = this.angularForce = 0;
    }

    resetConstraintVelocity() {
        const b = this;
        const vlambda = b.vlambda;
        vlambda.set(0,0);
        b.wlambda = 0;
    }

    addConstraintVelocity() {
        const b = this;
        const v = b.velocity;
        v.add(b.vlambda);
        b.angularVelocity += b.wlambda;
    }

    /**
     * Apply damping, see <a href="http://code.google.com/p/bullet/issues/detail?id=74">this</a> for details.
     * @method applyDamping
     * @param  {number} dt Current time step
     */
    applyDamping(dt) {
        if(this.type === Body.DYNAMIC){ // Only for dynamic bodies
            const v = this.velocity;
            v.scale((1 - this.damping) ** dt);
            this.angularVelocity *= (1 - this.angularDamping) ** dt;
        }
    }

    /**
     * Wake the body up. Normally you should not need this, as the body is automatically awoken at events such as collisions.
     * Sets the sleepState to {{#crossLink "Body/AWAKE:property"}}Body.AWAKE{{/crossLink}} and emits the wakeUp event if the body wasn't awake before.
     * @method wakeUp
     */
    wakeUp() {
        const s = this.sleepState;
        this.sleepState = Body.AWAKE;
        this.idleTime = 0;
        if(s !== Body.AWAKE){
            this.emit(wakeUpEvent);
        }
    }

    /**
     * Force body sleep
     * @method sleep
     */
    sleep() {
        this.sleepState = Body.SLEEPING;
        this.angularVelocity = this.angularForce = 0;
        this.velocity.set(0,0);
        this.force.set(0,0);
        this.emit(sleepEvent);
    }

    /**
     * Called every timestep to update internal sleep timer and change sleep state if needed.
     * @method sleepTick
     * @param {number} time The world time in seconds
     * @param {boolean} dontSleep
     * @param {number} dt
     */
    sleepTick(time, dontSleep, dt) {
        if(!this.allowSleep || this.type === Body.SLEEPING){
            return;
        }

        this.wantsToSleep = false;

        const speedSquared = this.velocity.squaredLength() + this.angularVelocity ** 2;
        const speedLimitSquared = this.sleepSpeedLimit ** 2;

        // Add to idle time
        if(speedSquared >= speedLimitSquared){
            this.idleTime = 0;
            this.sleepState = Body.AWAKE;
        } else {
            this.idleTime += dt;
            if(this.sleepState !== Body.SLEEPY){
                this.sleepState = Body.SLEEPY;
                this.emit(sleepyEvent);
            }
        }

        if(this.idleTime > this.sleepTimeLimit){
            if(!dontSleep){
                this.sleep();
            } else {
                this.wantsToSleep = true;
            }
        }
    }

    /**
     * Check if the body is overlapping another body. Note that this method only works if the body was added to a World and if at least one step was taken.
     * @method overlaps
     * @param  {Body} body
     * @return {boolean}
     */
    overlaps(body) {
        return this.world.overlapKeeper.bodiesAreOverlapping(this, body);
    }

    /**
     * Move the body forward in time given its current velocity.
     * @method integrate
     * @param  {Number} dt
     */
    integrate(dt) {
        const minv = this.invMass;
        const f = this.force;
        const pos = this.position;
        const velo = this.velocity;

        // Save old position
        this.previousPosition.copy(this.position);
        this.previousAngle = this.angle;

        // Velocity update
        if(!this.fixedRotation){
            this.angularVelocity += this.angularForce * this.invInertia * dt;
        }
        f.scale(dt * minv, integrate_fhMinv);
        integrate_fhMinv.multiply(this.massMultiplier);
        velo.add(integrate_fhMinv);

        // CCD
        if(!this.integrateToTimeOfImpact(dt)){

            // Regular position update
            velo.scale(dt, integrate_velodt);
            pos.add(integrate_velodt);
            if(!this.fixedRotation){
                this.angle += this.angularVelocity * dt;
            }
        }

        this.aabbNeedsUpdate = true;
    }

    integrateToTimeOfImpact(dt) {

        if(this.ccdSpeedThreshold < 0 || this.velocity.squaredLength() < this.ccdSpeedThreshold ** 2){
            return false;
        }

        // Ignore all the ignored body pairs
        // This should probably be done somewhere else for optimization
        const ignoreBodies = [];
        const disabledPairs = this.world.disabledBodyCollisionPairs;
        for(let i = 0; i < disabledPairs.length; i+=2){
            const bodyA = disabledPairs[i];
            const bodyB = disabledPairs[i+1];
            if(bodyA === this){
                ignoreBodies.push(bodyB);
            } else if(bodyB === this){
                ignoreBodies.push(bodyA);
            }
        }

        this.velocity.normalize(direction);

        this.velocity.scale(dt, end);
        end.add(this.position);

        end.subtract(this.position, startToEnd);
        const startToEndAngle = this.angularVelocity * dt;
        const len = startToEnd.length();

        let timeOfImpact = 1;

        let hitBody;
        ray.from.copy(this.position);
        ray.to.copy(end);
        ray.update();
        for(var i=0; i<this.shapes.length; i++){
            const shape = this.shapes[i];
            result.reset();
            ray.collisionGroup = shape.collisionGroup;
            ray.collisionMask = shape.collisionMask;
            this.world.raycast(result, ray);
            hitBody = result.body;

            if(hitBody === this || ignoreBodies.includes(hitBody)){
                hitBody = null;
            }

            if(hitBody){
                break;
            }
        }

        if(!hitBody || !timeOfImpact){
            return false;
        }
        result.getHitPoint(end, ray);
        end.subtract(this.position, startToEnd);
        timeOfImpact = end.distanceTo(this.position) / len; // guess

        const rememberAngle = this.angle;
        rememberPosition.copy(this.position);

        // Got a start and end point. Approximate time of impact using binary search
        let iter = 0;
        let tmin = 0;
        let tmid = timeOfImpact;
        let tmax = 1;
        while (tmax >= tmin && iter < this.ccdIterations) {
            iter++;

            // calculate the midpoint
            tmid = (tmax + tmin) / 2;

            // Move the body to that point
            startToEnd.scale(tmid, integrate_velodt);
            rememberPosition.add(integrate_velodt, this.position);
            this.angle = rememberAngle + startToEndAngle * tmid;
            this.updateAABB();

            // check overlap
            const overlaps = this.aabb.overlaps(hitBody.aabb) && this.world.narrowphase.bodiesOverlap(this, hitBody, true);

            if (overlaps) {
                // change max to search lower interval
                tmax = tmid;
            } else {
                // change min to search upper interval
                tmin = tmid;
            }
        }

        timeOfImpact = tmax; // Need to guarantee overlap to resolve collisions

        this.position.copy(rememberPosition);
        this.angle = rememberAngle;

        // move to TOI
        startToEnd.scale(timeOfImpact, integrate_velodt);
        this.position.add(integrate_velodt);
        if(!this.fixedRotation){
            this.angle += startToEndAngle * timeOfImpact;
        }

        return true;
    }

    /**
     * Get velocity of a point in the body.
     * @method getVelocityAtPoint
     * @param  {Array} result A vector to store the result in
     * @param  {Array} relativePoint A world oriented vector, indicating the position of the point to get the velocity from
     * @return {Array} The result vector
     * @example
     *     var body = new Body({
     *         mass: 1,
     *         velocity: [1, 0],
     *         angularVelocity: 1
     *     });
     *     var result = [];
     *     var point = [1, 0];
     *     body.getVelocityAtPoint(result, point);
     *     console.log(result); // [1, 1]
     */
    getVelocityAtPoint(result, relativePoint) {
        relativePoint.crossVZ(this.angularVelocity, result);
        result.subtract(this.velocity);
        return result;
    }
}

/**
 * @event sleepy
 */
const sleepyEvent = {
    type: "sleepy"
};

/**
 * @event sleep
 */
const sleepEvent = {
    type: "sleep"
};

/**
 * @event wakeup
 */
const wakeUpEvent = {
    type: "wakeup"
};

const shapeAABB = new AABB();
const tmp = new Vec2();

/**
 * Apply force to a point relative to the center of mass of the body. This could for example be a point on the Body surface. Applying force this way will add to Body.force and Body.angularForce.
 * @method applyForceLocal
 * @param  {Array} localForce The force vector to add, oriented in local body space.
 * @param  {Array} [localPoint] A point relative to the body in local body space. If not given, it is set to zero and all of the force will be exerted on the center of mass.
 * @example
 *     var body = new Body({ mass: 1 });
 *     var localPoint = [1, 0]; // x=1 locally in the body
 *     var localForce = [0, 1]; // up, locally in the body
 *     body.applyForceLocal(localForce, localPoint);
 *     console.log(body.force); // [0, 1]
 *     console.log(body.angularForce); // 1
 */
const Body_applyForce_forceWorld = new Vec2();
const Body_applyForce_pointWorld = new Vec2();
const Body_applyForce_pointLocal = new Vec2();

/**
 * Apply impulse to a point relative to the body. This could for example be a point on the Body surface. An impulse is a force added to a body during a short period of time (impulse = force * time). Impulses will be added to Body.velocity and Body.angularVelocity.
 * @method applyImpulse
 * @param  {Array} impulseVector The impulse vector to add, oriented in world space.
 * @param  {Array} [relativePoint] A point relative to the body in world space. If not given, it is set to zero and all of the impulse will be exerted on the center of mass.
 * @example
 *     var body = new Body({ mass: 1 });
 *     var relativePoint = [0, 0]; // center of the body
 *     var impulseVector = [0, 1]; // world up
 *     body.applyImpulse(impulseVector, relativePoint);
 */
const Body_applyImpulse_velo = new Vec2();

/**
 * Apply impulse to a point relative to the body. This could for example be a point on the Body surface. An impulse is a force added to a body during a short period of time (impulse = force * time). Impulses will be added to Body.velocity and Body.angularVelocity.
 * @method applyImpulseLocal
 * @param  {Array} localImpulse The impulse vector to add, oriented in local body space.
 * @param  {Array} [localPoint] A point relative to the body in local body space. If not given, it is set to zero and all of the impulse will be exerted on the center of mass.
 * @example
 *     var body = new Body({ mass: 1 });
 *     var localPoint = [1, 0]; // x=1, locally in the body
 *     var localImpulse = [0, 1]; // up, locally in the body
 *     body.applyImpulseLocal(localImpulse, localPoint);
 *     console.log(body.velocity); // [1, 0]
 *     console.log(body.angularVelocity); // 1
 */
const Body_applyImpulse_impulseWorld = new Vec2();
const Body_applyImpulse_pointWorld = new Vec2();
const Body_applyImpulse_pointLocal = new Vec2();
const adjustCenterOfMass_tmp2 = new Vec2();
const adjustCenterOfMass_tmp3 = new Vec2();
const adjustCenterOfMass_tmp4 = new Vec2();

const integrate_fhMinv = new Vec2();
const integrate_velodt = new Vec2();

const result = new RaycastResult();
const ray = new Ray()
ray.mode = RAY_MODES.CLOSEST
ray.skipBackfaces = true
const direction = new Vec2();
const end = new Vec2();
const startToEnd = new Vec2();
const rememberPosition = new Vec2();

