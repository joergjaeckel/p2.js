/**
 * Defines a physics material.
 */
export class Material {
    /** Material id. */
    id: number

    static idCounter = 0

    constructor() {
        this.id = Material.idCounter++
    }
}
