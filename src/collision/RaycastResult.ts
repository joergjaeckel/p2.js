import { Vec2 } from '../math/Vec2'
import { Ray } from '../collision/Ray'
import type { Body } from '../objects/Body'
import type { Shape } from '../shapes/Shape'

/**
 * Storage for Ray casting data
 */
export class RaycastResult {

	/**
	 * The normal of the hit, oriented in world space.
	 * @property {array} normal
	 */
	normal: Vec2

	/**
	 * The hit shape, or null.
	 * @property {Shape} shape
	 */
	shape: Shape | null

	/**
	 * The hit body, or null.
	 * @property {Body} body
	 */
	body: Body | null

	/**
	 * The index of the hit triangle, if the hit shape was indexable.
	 * @property {number} faceIndex
	 * @default -1
	 */
	faceIndex = -1;

	/**
	 * Distance to the hit, as a fraction. 0 is at the "from" point, 1 is at the "to" point. Will be set to -1 if there was no hit yet.
	 * @property {number} fraction
	 * @default -1
	 */
	fraction = -1;

	/**
	 * If the ray should stop traversing.
	 * @readonly
	 * @property {Boolean} isStopped
	 */
	isStopped = false;

	constructor() {
		this.normal = new Vec2()
		this.shape = null
		this.body = null
	}

	/**
	 * Reset all result data. Must be done before re-using the result object.
	 * @method reset
	 */
	reset(): void {
		this.normal.setZero();
		this.shape = null;
		this.body = null;
		this.faceIndex = -1;
		this.fraction = -1;
		this.isStopped = false;
	};

	/**
	 * Get the distance to the hit point.
	 */
	getHitDistance(ray) {
		return ray.from.distanceTo(ray.to) * this.fraction;
	};

	/**
	 * Returns true if the ray hit something since the last reset().
	 */
	hasHit() {
		return this.fraction !== -1;
	};

	/**
	 * Get world hit point.
	 */
	getHitPoint(out, ray) {
		ray.from.lerp(ray.to, this.fraction, out);
	};

	/**
	 * Can be called while iterating over hits to stop searching for hit points.
	 */
	stop(){
		this.isStopped = true;
	};

	shouldStop(ray){
		return this.isStopped || (this.fraction !== -1 && ray.mode === Ray.ANY);
	};

	/**
	 * Set result data.
	 */
	set(
		normal: Vec2,
		shape: Shape,
		body: Body,
		fraction: number,
		faceIndex: number
	){
		this.normal.copy(normal);
		this.shape = shape;
		this.body = body;
		this.fraction = fraction;
		this.faceIndex = faceIndex;
	};
}
