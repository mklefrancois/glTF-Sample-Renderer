import { quat, vec3, mat4 } from "gl-matrix";

class PhysicsUtils {
    /**
     * Returns the cumulative scale and scale axis from the node up to the root,
     * which can be used to properly scale physics shapes
     * @param {gltfNode} node
     * @returns {{scale: vec3, scaleAxis: quat}}
     */
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

    /**
     * Recursively propagates a parent world transform down the node hierarchy,
     * computing and storing the scaled physics transform for each non-motion node.
     * Stops traversal at nodes that carry motion data.
     *
     * @param {glTF} gltf - The glTF asset containing the full node array.
     * @param {gltfNode} node - The current node to process.
     * @param {Float32Array} parentTransform - The 4x4 world transform of the parent node.
     */
    static applyTransformRecursively(gltf, node, parentTransform) {
        if (node.extensions?.KHR_physics_rigid_bodies?.motion !== undefined) {
            return;
        }
        const localTransform = node.getLocalTransform();
        const globalTransform = mat4.create();
        mat4.multiply(globalTransform, parentTransform, localTransform);
        node.scaledPhysicsTransform = globalTransform;
        for (const childIndex of node.children) {
            const childNode = gltf.nodes[childIndex];
            this.applyTransformRecursively(gltf, childNode, globalTransform);
        }
    }

    /**
     * Checks if the joint space of a joint node has changed by traversing up the hierarchy to find any dirty transforms.
     * @param {gltfNode} jointNode
     * @returns {boolean}
     */
    static hasJointSpaceChanged(jointNode) {
        if (jointNode.dirtyTransform === false) {
            return false;
        }
        let currentNode = jointNode;
        while (
            currentNode !== undefined &&
            currentNode.extensions?.KHR_physics_rigid_bodies?.motion === undefined
        ) {
            if (currentNode.isLocalTransformDirty()) {
                return true;
            }
            currentNode = currentNode.parent;
        }
        return false;
    }

    /**
     * Recursively traverses the node hierarchy of a motion to find all colliders and triggers, and applies the custom function to them.
     * The custom function has the following signature:
     * function(gltf, node, collider/trigger, motionNode, computedWorldTransform, offsetChanged, scaleChanged, isTrigger, ...args)
     * offsetChanged and scaleChanged are cumulative values that indicate whether any node in the hierarchy has a dirty offset or scale,
     * which can be used to determine if the physics shape needs to be updated.
     * isTrigger indicates whether the current geometry is a trigger or a collider.
     *
     * @param {gltf} gltf
     * @param {gltfNode} node
     * @param {KHR_physics_rigid_bodies_collider | KHR_physics_rigid_bodies_trigger} collider
     * @param {gltfNode} motionNode
     * @param {boolean} offsetChanged
     * @param {boolean} scaleChanged
     * @param {Function} customFunction
     * @param {Array} args
     */
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
