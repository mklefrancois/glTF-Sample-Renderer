/* eslint-disable no-unused-vars */
import { quat, vec3 } from "gl-matrix";

class PhysicsInterface {
    constructor() {
        this.simpleShapes = [];
    }

    async initializeEngine() {}
    initializeSimulation(
        state,
        staticActors,
        kinematicActors,
        dynamicActors,
        jointNodes,
        triggerNodes,
        independentTriggerNodes,
        nodeToMotion,
        hasRuntimeAnimationTargets,
        staticMeshColliderCount,
        dynamicMeshColliderCount
    ) {}
    pauseSimulation() {}
    resumeSimulation() {}
    resetSimulation() {}
    stopSimulation() {}
    enableDebugColliders(enable) {}
    enableDebugJoints(enable) {}

    applyImpulse(nodeIndex, linearImpulse, angularImpulse) {}
    applyPointImpulse(nodeIndex, impulse, position) {}
    rayCast(rayStart, rayEnd) {}

    generateBox(x, y, z, scale, scaleAxis, reference) {}
    generateCapsule(height, radiusTop, radiusBottom, scale, scaleAxis, reference) {}
    generateCylinder(height, radiusTop, radiusBottom, scale, scaleAxis, reference) {}
    generateSphere(radius, scale, scaleAxis, reference) {}
    generatePlane(width, height, doubleSided, scale, scaleAxis, reference) {}
    generateSimpleShape(
        shape,
        scale = vec3.fromValues(1, 1, 1),
        scaleAxis = quat.create(),
        reference = undefined
    ) {
        switch (shape.type) {
            case "box":
                return this.generateBox(
                    shape.box.size[0],
                    shape.box.size[1],
                    shape.box.size[2],
                    scale,
                    scaleAxis,
                    reference
                );
            case "capsule":
                return this.generateCapsule(
                    shape.capsule.height,
                    shape.capsule.radiusTop,
                    shape.capsule.radiusBottom,
                    scale,
                    scaleAxis,
                    reference
                );
            case "cylinder":
                return this.generateCylinder(
                    shape.cylinder.height,
                    shape.cylinder.radiusTop,
                    shape.cylinder.radiusBottom,
                    scale,
                    scaleAxis,
                    reference
                );
            case "sphere":
                return this.generateSphere(shape.sphere.radius, scale, scaleAxis, reference);
            case "plane":
                return this.generatePlane(
                    shape.plane.width,
                    shape.plane.height,
                    shape.plane.doubleSided,
                    scale,
                    scaleAxis,
                    reference
                );
        }
    }

    generateSimpleShapes(gltf) {
        this.simpleShapes = [];
        if (gltf?.extensions?.KHR_implicit_shapes === undefined) {
            return;
        }
        for (const shape of gltf.extensions.KHR_implicit_shapes.shapes) {
            this.simpleShapes.push(this.generateSimpleShape(shape));
        }
    }

    updateActorTransform(node) {}
    updatePhysicsJoint(state, jointNode) {}
}

export { PhysicsInterface };
