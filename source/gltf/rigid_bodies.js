import { GltfObject } from "./gltf_object";
import { objectsFromJsons } from "./utils";
import { quat } from "gl-matrix";

class KHR_physics_rigid_bodies extends GltfObject {
    static animatedProperties = [];
    static readonlyAnimatedProperties = ["physicsMaterials", "collisionFilters", "physicsJoints"];
    constructor() {
        super();
        this.physicsMaterials = [];
        this.collisionFilters = [];
        this.physicsJoints = [];
    }
    fromJson(json) {
        super.fromJson(json);
        this.physicsMaterials = objectsFromJsons(json.physicsMaterials, gltfPhysicsMaterial);
        this.collisionFilters = objectsFromJsons(json.collisionFilters, gltfCollisionFilter);
        this.physicsJoints = objectsFromJsons(json.physicsJoints, gltfPhysicsJoint);
    }
}

class gltfPhysicsMaterial extends GltfObject {
    static animatedProperties = ["staticFriction", "dynamicFriction", "restitution"];
    constructor() {
        super();
        this.staticFriction = 0.6;
        this.dynamicFriction = 0.6;
        this.restitution = 0;
        this.frictionCombine = undefined;
        this.restitutionCombine = undefined;
    }
}

class gltfCollisionFilter extends GltfObject {
    static animatedProperties = [];
    constructor() {
        super();
        this.collisionSystems = [];
        this.collideWithSystems = [];
        this.notCollideWithSystems = [];
    }
}

class simplifiedPhysicsJoint {
    constructor(limits, drives) {
        this.limits = limits;
        this.drives = drives;

        this.twistLimit = undefined;
        this.swingLimit1 = undefined;
        this.swingLimit2 = undefined;
        this.localRotation = quat.create();
        this.twistAxis = 0; // 0 = X, 1 = Y, 2 = Z
        this.isCylindrical = false;

        const freeAxes = new Set([0, 1, 2]);
        const limitAxes = new Map();
        const fixedAxes = new Map();

        for (const limit of this.limits) {
            if (limit.angularAxes !== undefined) {
                for (const axis of limit.angularAxes) {
                    if (limit.min === 0 && limit.max === 0) {
                        fixedAxes.set(axis, limit);
                    } else {
                        limitAxes.set(axis, limit);
                    }
                    freeAxes.delete(axis);
                }
                if (limit.angularAxes.length > 1) {
                    this.isCylindrical = true;
                }
            }
        }

        // Handle cylindrical joints (cone/ellipse limits)
        if (this.isCylindrical) {
            this._handleCylindricalLimits(limitAxes, fixedAxes);
            return;
        }

        if (freeAxes.size === 0) {
            // All axes are constrained
            if (limitAxes.size === 0) {
                // All axes are fixed (locked)
                this.twistLimit = fixedAxes.get(0);
                this.swingLimit1 = fixedAxes.get(1);
                this.swingLimit2 = fixedAxes.get(2);
            } else {
                // Mix of fixed and limited axes
                this._handleMixedConstraints(limitAxes, fixedAxes);
            }
        } else if (freeAxes.size === 1) {
            // Two axes are constrained, one is free
            const freeAxis = Array.from(freeAxes)[0];
            this._handleTwoConstrainedAxes(limitAxes, fixedAxes, freeAxis);
        } else if (freeAxes.size === 2) {
            // One axis is constrained, two are free
            const constrainedAxis = [0, 1, 2].find((axis) => !freeAxes.has(axis));
            this._handleOneConstrainedAxis(limitAxes, fixedAxes, constrainedAxis);
        }
    }

    getRotatedAxisAndSign(axis) {
        let result = {
            axis: axis,
            sign: 1
        };
        if (this.twistAxis === 0) {
            return result; // No rotation needed
        }
        if (this.twistAxis === 1) {
            if (axis === 0) {
                result.axis = 1;
            } else if (axis === 1) {
                result.axis = 0;
                result.sign = -1;
            } else {
                result.axis = 2;
            }
        } else {
            if (axis === 0) {
                result.axis = 2;
                result.sign = -1;
            } else if (axis === 1) {
                result.axis = 1;
            } else {
                result.axis = 0;
            }
        }
        return result;
    }

    _handleMixedConstraints(limitAxes, fixedAxes) {
        // Find the axis with the largest angular range to use as twist
        let maxAxis = -1;
        let maxRange = 0;

        for (const [axis, limit] of limitAxes.entries()) {
            const range = limit.max - limit.min;
            if (range > maxRange) {
                maxRange = range;
                maxAxis = axis;
            }
        }

        if (maxAxis === -1) {
            // No limited axes, all are fixed
            this.twistLimit = fixedAxes.get(0);
            this.swingLimit1 = fixedAxes.get(1);
            this.swingLimit2 = fixedAxes.get(2);
            return;
        }

        // Use the axis with largest range as twist axis
        this._assignLimitsWithTwistAxis(limitAxes, fixedAxes, maxAxis);
    }

    _handleTwoConstrainedAxes(limitAxes, fixedAxes, freeAxis) {
        // Two constrained axes should use swing limits (cone/pyramid)
        // The free axis becomes the twist axis
        const constrainedAxes = [0, 1, 2].filter((axis) => axis !== freeAxis);

        // Calculate local rotation to align free axis with PhysX twist axis (X-axis)
        this.localRotation = this._calculateLocalRotation(freeAxis);

        // Free axis becomes twist axis (may be free or have some constraint)
        this.twistLimit = limitAxes.get(freeAxis) || fixedAxes.get(freeAxis);

        // Constrained axes become swing limits
        this.swingLimit1 = limitAxes.get(constrainedAxes[0]) || fixedAxes.get(constrainedAxes[0]);
        this.swingLimit2 = limitAxes.get(constrainedAxes[1]) || fixedAxes.get(constrainedAxes[1]);
    }

    _handleOneConstrainedAxis(limitAxes, fixedAxes, constrainedAxis) {
        // Use the constrained axis as twist axis
        this._assignLimitsWithTwistAxis(limitAxes, fixedAxes, constrainedAxis);
    }

    _assignLimitsWithTwistAxis(limitAxes, fixedAxes, twistAxis) {
        // Calculate local rotation to align twist axis with PhysX convention (X-axis)
        this.localRotation = this._calculateLocalRotation(twistAxis);

        // Assign limits based on the chosen twist axis
        this.twistLimit = limitAxes.get(twistAxis) || fixedAxes.get(twistAxis);

        // Assign swing limits for the other two axes
        const swingAxes = [0, 1, 2].filter((axis) => axis !== twistAxis);
        this.swingLimit1 = limitAxes.get(swingAxes[0]) || fixedAxes.get(swingAxes[0]);
        this.swingLimit2 = limitAxes.get(swingAxes[1]) || fixedAxes.get(swingAxes[1]);
    }

    _calculateLocalRotation(twistAxis) {
        // Calculate rotation to align the chosen twist axis with PhysX X-axis
        const rotation = quat.create();
        this.twistAxis = twistAxis;

        switch (twistAxis) {
            case 0: // X-axis is already aligned
                quat.identity(rotation);
                break;
            case 1: // Y-axis -> rotate X-axis to align with Y-axis
                // Rotate 90 degrees around Z-axis
                quat.fromEuler(rotation, 0, 0, 90);
                break;
            case 2: // Z-axis -> rotate X-axis to align with Z-axis
                // Rotate -90 degrees around Y-axis
                quat.fromEuler(rotation, 0, -90, 0);
                break;
        }

        return rotation;
    }

    _handleCylindricalLimits(limitAxes, fixedAxes) {
        // Handle limits that constrain multiple axes together (cone/ellipse)
        // Find the limit that affects multiple axes
        for (const [axis, limit] of limitAxes.entries()) {
            if (limit.angularAxes && limit.angularAxes.length > 1) {
                // This is a cone/ellipse limit
                const affectedAxes = limit.angularAxes;
                const freeAxis = [0, 1, 2].find((axis) => !affectedAxes.includes(axis));

                if (freeAxis !== undefined) {
                    // Free axis becomes twist
                    this.localRotation = this._calculateLocalRotation(freeAxis);
                    this.twistLimit = limitAxes.get(freeAxis) || fixedAxes.get(freeAxis);

                    // Cone limit affects both swing axes equally
                    this.swingLimit1 = limit;
                    this.swingLimit2 = limit;
                } else {
                    // All axes are in the cone - use first axis as twist
                    this.swingLimit1 = limit;
                    this.swingLimit2 = limit;
                }
                break;
            }
        }
    }
}

class gltfPhysicsJoint extends GltfObject {
    static animatedProperties = [];
    constructor() {
        super();
        this.limits = [];
        this.drives = [];

        // non glTF
        this.simplifiedPhysicsJoints = [];
    }

    _getUniqueDrives(drivesCopy) {
        const definedLinearDrives = new Set();
        const definedAngularDrives = new Set();
        const result = [];
        for (let i = drivesCopy.length - 1; i >= 0; i--) {
            if (drivesCopy[i].type === "linear" && definedLinearDrives.has(drivesCopy[i].axis)) {
                continue;
            }
            if (drivesCopy[i].type === "angular" && definedAngularDrives.has(drivesCopy[i].axis)) {
                continue;
            }
            if (drivesCopy[i].type === "linear") {
                definedLinearDrives.add(drivesCopy[i].axis);
                result.push(drivesCopy[i]);
                drivesCopy.splice(i, 1);
            } else {
                definedAngularDrives.add(drivesCopy[i].axis);
                result.push(drivesCopy[i]);
                drivesCopy.splice(i, 1);
            }
        }
        return result;
    }

    fromJson(json) {
        super.fromJson(json);
        this.limits = objectsFromJsons(json.limits, gltfPhysicsJointLimit);
        this.drives = objectsFromJsons(json.drives, gltfPhysicsJointDrive);

        const definedLinearAxes = new Set();
        const definedAngularAxes = new Set();
        let currentLimits = [];
        const drivesCopy = this.drives.slice();

        let needToCreateNewJoint = false;
        for (const limit of this.limits) {
            for (const axis of limit.angularAxes || []) {
                if (definedAngularAxes.has(axis)) {
                    needToCreateNewJoint = true;
                }
            }
            for (const axis of limit.linearAxes || []) {
                if (definedLinearAxes.has(axis)) {
                    needToCreateNewJoint = true;
                }
            }
            if (needToCreateNewJoint) {
                const drives = this._getUniqueDrives(drivesCopy);
                this.simplifiedPhysicsJoints.push(
                    new simplifiedPhysicsJoint(currentLimits, drives)
                );
                currentLimits = [];
                definedLinearAxes.clear();
                definedAngularAxes.clear();
                needToCreateNewJoint = false;
            }
            currentLimits.push(limit);
            for (const axis of limit.angularAxes || []) {
                definedAngularAxes.add(axis);
            }
            for (const axis of limit.linearAxes || []) {
                definedLinearAxes.add(axis);
            }
        }
        if (currentLimits.length > 0) {
            const drives = this._getUniqueDrives(drivesCopy);
            this.simplifiedPhysicsJoints.push(new simplifiedPhysicsJoint(currentLimits, drives));
        }
        while (drivesCopy.length > 0) {
            const drives = this._getUniqueDrives(drivesCopy);
            this.simplifiedPhysicsJoints.push(new simplifiedPhysicsJoint([], drives));
        }
    }
}

class gltfPhysicsJointLimit extends GltfObject {
    static animatedProperties = ["min", "max", "stiffness", "damping"];
    constructor() {
        super();
        this.min = undefined;
        this.max = undefined;
        this.stiffness = undefined;
        this.damping = 0;
        this.linearAxes = undefined;
        this.angularAxes = undefined;
    }
}

class gltfPhysicsJointDrive extends GltfObject {
    static animatedProperties = [
        "maxForce",
        "positionTarget",
        "velocityTarget",
        "stiffness",
        "damping"
    ];
    constructor() {
        super();
        this.type = undefined;
        this.mode = undefined;
        this.axis = undefined;
        this.maxForce = undefined;
        this.positionTarget = undefined;
        this.velocityTarget = undefined;
        this.stiffness = 0;
        this.damping = 0;
    }
}

export {
    KHR_physics_rigid_bodies,
    gltfPhysicsMaterial,
    gltfCollisionFilter,
    gltfPhysicsJoint,
    gltfPhysicsJointLimit,
    gltfPhysicsJointDrive
};
