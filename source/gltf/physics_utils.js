import { quat, vec3 } from "gl-matrix";

class PhysicsUtils {
    static calculateScaleAndAxis(node) {
        const scaleFactor = vec3.clone(node.scale);
        let scaleRotation = quat.create();

        let currentNode = node.parentNode;
        const currentRotation = quat.clone(node.rotation);

        while (currentNode !== undefined) {
            if (vec3.equals(currentNode.scale, vec3.fromValues(1, 1, 1)) === false) {
                const localScale = currentNode.scale;
                vec3.transformQuat(localScale, currentNode.scale, scaleRotation);
                vec3.multiply(scaleFactor, scaleFactor, localScale);
                scaleRotation = quat.clone(currentRotation);
            }
            const nextRotation = quat.clone(currentNode.rotation);
            quat.multiply(currentRotation, currentRotation, nextRotation);
            currentNode = currentNode.parentNode;
        }
        return { scale: scaleFactor, scaleAxis: scaleRotation };
    }

    /**
     * Converts triangle strip indices to triangle list indices
     * @param {Uint32Array|Array} stripIndices - The triangle strip indices
     * @returns {Uint32Array} - Triangle list indices
     */
    static convertTriangleStripToTriangles(stripIndices) {
        if (stripIndices.length < 3) {
            return new Uint32Array(0);
        }

        const triangleCount = stripIndices.length - 2;
        const triangleIndices = new Uint32Array(triangleCount * 3);
        let triangleIndex = 0;

        for (let i = 0; i < triangleCount; i++) {
            if (i % 2 === 0) {
                // Even triangle: maintain winding order
                triangleIndices[triangleIndex++] = stripIndices[i];
                triangleIndices[triangleIndex++] = stripIndices[i + 1];
                triangleIndices[triangleIndex++] = stripIndices[i + 2];
            } else {
                // Odd triangle: reverse winding order
                triangleIndices[triangleIndex++] = stripIndices[i];
                triangleIndices[triangleIndex++] = stripIndices[i + 2];
                triangleIndices[triangleIndex++] = stripIndices[i + 1];
            }
        }

        return triangleIndices;
    }

    /**
     * Converts triangle fan indices to triangle list indices
     * @param {Uint32Array|Array} fanIndices - The triangle fan indices
     * @returns {Uint32Array} - Triangle list indices
     */
    static convertTriangleFanToTriangles(fanIndices) {
        if (fanIndices.length < 3) {
            return new Uint32Array(0);
        }

        const triangleCount = fanIndices.length - 2;
        const triangleIndices = new Uint32Array(triangleCount * 3);
        let triangleIndex = 0;

        const centerVertex = fanIndices[0];

        for (let i = 1; i < fanIndices.length - 1; i++) {
            triangleIndices[triangleIndex++] = fanIndices[i];
            triangleIndices[triangleIndex++] = fanIndices[i + 1];
            triangleIndices[triangleIndex++] = centerVertex;
        }

        return triangleIndices;
    }

    static recurseCollider(
        gltf,
        node,
        collider,
        motionNode,
        offsetChanged,
        scaleChanged,
        customFunction,
        args = []
    ) {
        // Do not add other motion bodies' shapes to this actor
        if (node.extensions?.KHR_physics_rigid_bodies?.motion !== undefined) {
            return;
        }

        const computedWorldTransform = node.worldTransform;
        if (node.animatedPropertyObjects.scale.dirty) {
            scaleChanged = true;
        }
        if (node.isLocalTransformDirty()) {
            offsetChanged = true;
        }

        // Found a collider geometry
        if (
            node.extensions?.KHR_physics_rigid_bodies?.collider?.geometry?.mesh !== undefined ||
            node.extensions?.KHR_physics_rigid_bodies?.collider?.geometry?.shape !== undefined
        ) {
            customFunction(
                gltf,
                node,
                node.extensions.KHR_physics_rigid_bodies.collider,
                motionNode,
                computedWorldTransform,
                offsetChanged,
                scaleChanged,
                false,
                ...args
            );
        }

        // Found a trigger
        if (
            node.extensions?.KHR_physics_rigid_bodies?.trigger?.geometry?.mesh !== undefined ||
            node.extensions?.KHR_physics_rigid_bodies?.trigger?.geometry?.shape !== undefined
        ) {
            customFunction(
                gltf,
                node,
                node.extensions.KHR_physics_rigid_bodies.trigger,
                motionNode,
                computedWorldTransform,
                offsetChanged,
                scaleChanged,
                true,
                ...args
            );
        }

        for (const childIndex of node.children) {
            const childNode = gltf.nodes[childIndex];
            this.recurseCollider(
                gltf,
                childNode,
                collider,
                motionNode,
                offsetChanged,
                scaleChanged,
                customFunction,
                args
            );
        }
    }
}

export { PhysicsUtils };
