/*function Shape(options){


TODO: huh what to do with this. cannon-es doesnt have some equivalent

    this.angle = options.angle || 0;


    if(this.type){
        this.updateBoundingRadius();
    }

    this.updateArea();
}*/

import type { Body } from '../objects/Body'
import type { Material } from '../material/Material'
import {Vec2} from "../math/Vec2";

/**
 * The available shape types.
 */
export const SHAPE_TYPES = {
    /** CIRCLE */
    CIRCLE: 1,
    /** PARTICLE */
    PARTICLE: 2,
    /** PLANE */
    PLANE: 4,
    /** CONVEX */
    CONVEX: 8,
    /** LINE */
    LINE: 16,
    /** BOX */
    BOX: 32,
    /** CAPSULE */
    CAPSULE: 64,
    /** HEIGHTFIELD */
    HEIGHTFIELD: 128,
} as const

/**
 * ShapeType
 */
export type ShapeType = typeof SHAPE_TYPES[keyof typeof SHAPE_TYPES]

export type ShapeOptions = ConstructorParameters<typeof Shape>[0]

/**
 * Base class for shapes
 */
export class Shape {
    /**
     * Identifier of the Shape.
     */
    id: number

    /**
     * The type of this shape. Must be set to an int > 0 by subclasses.
     */
    type: ShapeType | 0

    /**
     * The local bounding radius of this shape.
     */
    boundingRadius: number

    /**
     * Whether to produce contact forces when in contact with other bodies. Note that contacts will be generated, but they will be disabled.
     * @default true
     */
    collisionResponse: boolean

    /**
     * @default 1
     */
    collisionGroup: number

    /**
     * @default -1
     */
    collisionMask: number

    /**
     * World space position of the shape.
     */
    position: Vec2

    /**
     * World space orientation of the shape.
     */
    angle: number

    /**
     * Optional material of the shape that regulates contact properties.
     */
    material: Material | null

    /**
     * The body to which the shape is added to.
     */
    body: Body | null

    area: number

    /**
     * Set to true if you want this shape to be a sensor. A sensor does not generate contacts, but it still reports contact events. This is good if you want to know if a shape is overlapping another shape, without them generating contacts.
     * @default false
     */
    sensor: boolean

    static idCounter = 0

    /**
     * All the Shape types.
     */
    static types = SHAPE_TYPES

    constructor(
        options: {
            /**
             * The type of this shape.
             */
            type?: ShapeType
            /**
             * Whether to produce contact forces when in contact with other bodies.
             * @default true
             */
            collisionResponse?: boolean
            /**
             * @default 1
             */
            collisionFilterGroup?: number
            /**
             * @default -1
             */
            collisionFilterMask?: number

            position?: Vec2

            angle?: number
            /**
             * Optional material of the shape that regulates contact properties.
             * @default null
             * @todo check this, the material is passed to the body, right?
             */
            material?: Material
        } = {}
    ) {
        this.id = Shape.idCounter++
        this.type = options.type || 0
        this.boundingRadius = 0
        this.collisionResponse = options.collisionResponse ? options.collisionResponse : true
        this.collisionGroup = options.collisionFilterGroup !== undefined ? options.collisionFilterGroup : 1
        this.collisionMask = options.collisionFilterMask !== undefined ? options.collisionFilterMask : -1
        this.position = options.position || new Vec2();
        this.angle = options.angle || 0;
        this.material = options.material ? options.material : null
        this.body = null
        this.area = 0
        this.sensor = false
    }

    /**
     * Should return the moment of inertia around the Z axis of the body. See <a href="http://en.wikipedia.org/wiki/List_of_moments_of_inertia">Wikipedia's list of moments of inertia</a>.
     * @method computeMomentOfInertia
     * @return {Number} If the inertia is infinity or if the object simply isn't possible to rotate, return 0.
     */
    computeMomentOfInertia(): number {
        throw `computeMomentOfInertia() not implemented for shape type ${this.type}`
    }

    /**
     * Returns the bounding circle radius of this shape.
     * @method updateBoundingRadius
     * @return {Number}
     */
    updateBoundingRadius(): number {
        throw `updateBoundingRadius() not implemented for shape type ${this.type}`
    }

    /**
     * Update the .area property of the shape.
     * @method updateArea
     */
    updateArea() {
        throw `updateArea() not implemented for shape type ${this.type}`
    }

    /**
     * Compute the world axis-aligned bounding box (AABB) of this shape.
     * @method computeAABB
     * @param  {AABB} out The resulting AABB.
     * @param  {Array} position World position of the shape.
     * @param  {Number} angle World angle of the shape.
     */
    computeAABB(out, position, angle){
        throw `computeAABB() not implemented for shape type ${this.type}`
    }

    /**
     * Perform raycasting on this shape.
     * @method raycast
     * @param  {RayResult} result Where to store the resulting data.
     * @param  {Ray} ray The Ray that you want to use for raycasting.
     * @param  {array} position World position of the shape (the .position property will be ignored).
     * @param  {number} angle World angle of the shape (the .angle property will be ignored).
     */
    raycast(/*result, ray, position, angle*/){
        throw `raycast() not implemented for shape type ${this.type}`
    }
}
