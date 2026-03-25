import { getAnimatedIndices } from "../gltf/gltf_utils";
import { NvidiaPhysicsInterface } from "../PhysicsEngines/PhysX";
import { PhysicsUtils } from "../gltf/physics_utils";

class PhysicsController {
    constructor() {
        this.engine = undefined;
        this.staticActors = [];
        this.kinematicActors = []; // This list is not updated if a dynamic actor is switched to kinematic at runtime
        this.dynamicActors = [];
        this.triggerNodes = [];
        this.independentTriggerNodes = []; // Trigger nodes that are not not part of another actor
        this.compoundTriggerNodes = new Map(); // Map of compound trigger node index to set of included colliders
        this.triggerToCompound = new Map(); // Map of trigger node index to compound trigger node index
        this.nodeToMotion = new Map();
        this.jointNodes = [];
        this.morphedColliders = [];
        this.skinnedColliders = [];
        this.hasRuntimeAnimationTargets = false;
        this.morphWeights = new Map();

        this.playing = false;
        this.enabled = false;
        this.simulationStepTime = 1 / 60;
        this.timeAccumulator = 0;
        this.pauseTime = undefined;
        this.skipFrames = 2; // Skip the first two simulation frames to allow engine to initialize
        this.loading = false;
    }

    calculateMorphColliders(gltf) {
        for (const node of this.morphedColliders) {
            const mesh = gltf.meshes[node.mesh];
            let morphWeights = node.weights ?? mesh.weights;
            if (morphWeights === undefined) {
                continue;
            }
            morphWeights = morphWeights.slice();
            const oldMorphWeights = this.morphWeights.get(node.gltfObjectIndex);

            // Check if morph weights have changed
            if (
                oldMorphWeights !== undefined &&
                oldMorphWeights.length === morphWeights.length &&
                oldMorphWeights.every((value, index) => value === morphWeights[index])
            ) {
                continue;
            }

            this.morphWeights.set(node.gltfObjectIndex, morphWeights);

            const vertices = new Float32Array();

            for (const primitive of mesh.primitives) {
                const positionAccessor = gltf.accessors[primitive.attributes.POSITION];
                const positionData = positionAccessor.getNormalizedDeinterlacedView(gltf);
                const morphData = [];
                for (let i = 0; i < morphWeights.length; i++) {
                    const morphAccessor = gltf.accessors[primitive.targets[i].POSITION];
                    morphData.push(morphAccessor.getNormalizedDeinterlacedView(gltf));
                }

                // Calculate morphed vertex positions on CPU
                for (let i = 0; i < positionData.length; i++) {
                    let position = positionData[i];
                    for (let j = 0; j < morphWeights.length; j++) {
                        const morphValue = morphData[j];
                        position += morphValue[i] * morphWeights[j];
                    }
                    vertices.push(position);
                }
            }

            this.engine.updateMorphedColliderGeometry(node, vertices);
        }
    }

    async initializeEngine(engine) {
        if (engine === "NvidiaPhysX") {
            this.engine = new NvidiaPhysicsInterface();
            await this.engine.initializeEngine();
        }
    }

    loadScene(state, sceneIndex) {
        this.resetScene(state.gltf);
        if (
            state.gltf.extensionsUsed === undefined ||
            state.gltf.extensionsUsed.includes("KHR_physics_rigid_bodies") === false
        ) {
            this.enabled = false;
            return;
        }
        const scene = state.gltf.scenes[sceneIndex];
        if (!scene.nodes) {
            this.enabled = false;
            return;
        }
        this.skipFrames = 2;
        this.loading = true;

        // Morphing physics colliders was dropped from the spec.
        // const morphedNodeIndices = getMorphedNodeIndices(state.gltf);
        const result = getAnimatedIndices(state.gltf, "/nodes/", [
            "translation",
            "rotation",
            "scale"
        ]);
        let dynamicMeshColliderCount = 0;
        let staticMeshColliderCount = 0;
        this.hasRuntimeAnimationTargets = result.runtimeChanges;
        const gatherRigidBodies = (nodeIndex, currentRigidBody) => {
            let parentRigidBody = currentRigidBody;
            const node = state.gltf.nodes[nodeIndex];
            const rigidBody = node.extensions?.KHR_physics_rigid_bodies;
            if (rigidBody) {
                if (rigidBody.motion) {
                    if (rigidBody.motion.isKinematic) {
                        this.kinematicActors.push(node);
                    } else {
                        this.dynamicActors.push(node);
                    }
                    parentRigidBody = node;
                } else if (currentRigidBody === undefined && rigidBody.collider !== undefined) {
                    this.staticActors.push(node);
                }
                if (rigidBody.collider?.geometry?.mesh !== undefined) {
                    if (!rigidBody.collider.geometry.convexHull) {
                        if (
                            parentRigidBody === undefined ||
                            parentRigidBody.extensions.KHR_physics_rigid_bodies.motion.isKinematic
                        ) {
                            staticMeshColliderCount++;
                        } else {
                            if (
                                currentRigidBody?.gltfObjectIndex !==
                                parentRigidBody.gltfObjectIndex
                            ) {
                                dynamicMeshColliderCount++;
                            }
                        }
                    }
                }
                if (rigidBody.joint !== undefined) {
                    this.jointNodes.push(node);
                }
                if (rigidBody.trigger !== undefined) {
                    if (rigidBody.trigger.nodes !== undefined) {
                        this.compoundTriggerNodes.set(node.gltfObjectIndex, {
                            previous: new Map(), //ref counting
                            added: new Set(),
                            removed: new Set()
                        });
                        for (const triggerNodeIndex of rigidBody.trigger.nodes) {
                            if (this.triggerToCompound.has(triggerNodeIndex)) {
                                this.triggerToCompound
                                    .get(triggerNodeIndex)
                                    .add(node.gltfObjectIndex);
                            } else {
                                this.triggerToCompound.set(
                                    triggerNodeIndex,
                                    new Set([node.gltfObjectIndex])
                                );
                            }
                        }
                    } else {
                        this.triggerNodes.push(node);
                        if (parentRigidBody === undefined) {
                            this.independentTriggerNodes.push(node);
                        }
                    }
                }
            }

            if (parentRigidBody !== undefined) {
                this.nodeToMotion.set(node.gltfObjectIndex, parentRigidBody.gltfObjectIndex);
            }
            for (const childIndex of node.children) {
                gatherRigidBodies(childIndex, parentRigidBody);
            }
        };

        for (const nodeIndex of scene.nodes) {
            gatherRigidBodies(nodeIndex, undefined);
        }
        if (
            !this.engine ||
            (this.staticActors.length === 0 &&
                this.kinematicActors.length === 0 &&
                this.dynamicActors.length === 0 &&
                this.triggerNodes.length === 0)
        ) {
            this.enabled = false;
            return;
        }
        this.enabled = true;
        this.engine.initializeSimulation(
            state,
            this.staticActors,
            this.kinematicActors,
            this.dynamicActors,
            this.jointNodes,
            this.triggerNodes,
            this.independentTriggerNodes,
            this.nodeToMotion,
            this.hasRuntimeAnimationTargets,
            staticMeshColliderCount,
            dynamicMeshColliderCount
        );
        this.loading = false;
        state.gltf.resetAllDirtyFlags();
        this.simulateStep(state, 0); // Simulate an initial step to ensure everything is up to date before rendering
    }

    resetScene(gltf) {
        this.staticActors = [];
        this.kinematicActors = [];
        this.dynamicActors = [];
        this.jointNodes = [];
        this.triggerNodes = [];
        this.independentTriggerNodes = [];
        this.nodeToMotion.clear();
        this.compoundTriggerNodes.clear();
        this.triggerToCompound.clear();
        this.morphedColliders = [];
        this.skinnedColliders = [];
        this.hasRuntimeAnimationTargets = false;
        this.morphWeights.clear();
        this.timeAccumulator = 0;
        for (const node of gltf?.nodes ?? []) {
            node.physicsTransform = undefined;
            node.scaledPhysicsTransform = undefined;
        }
        if (this.engine) {
            this.engine.resetSimulation();
        }
    }

    stopSimulation() {
        this.playing = false;
        this.enabled = false;
        if (this.engine) {
            this.engine.stopSimulation();
        }
    }

    resumeSimulation() {
        if (this.engine) {
            this.enabled = true;
            this.playing = true;
        }
    }

    pauseSimulation() {
        this.pauseTime = performance.now();
        this.enabled = true;
        this.playing = false;
    }

    simulateStep(state, deltaTime) {
        if (state === undefined) {
            return;
        }
        if (this.loading) {
            return;
        }
        if (this.skipFrames > 0) {
            this.skipFrames -= 1;
            return;
        }
        this.applyAnimations(state);
        this.timeAccumulator += deltaTime;
        if (this.pauseTime !== undefined) {
            this.timeAccumulator = this.simulationStepTime;
            if (this.playing) {
                this.pauseTime = undefined;
            }
        }
        if (
            this.enabled &&
            this.engine &&
            state &&
            this.timeAccumulator >= this.simulationStepTime * 0.9
        ) {
            this.engine.simulateStep(state, this.timeAccumulator);
            this.timeAccumulator = 0;
        }
    }

    updateColliders(state, node, isTrigger = false) {
        this.engine.updateActorTransform(node);

        let collider = undefined;
        if (isTrigger) {
            collider = node.extensions?.KHR_physics_rigid_bodies?.trigger;
        } else {
            collider = node.extensions?.KHR_physics_rigid_bodies?.collider;
        }

        if (collider?.geometry?.shape !== undefined || collider?.geometry?.mesh !== undefined) {
            this.engine.updateCollider(
                state.gltf,
                node,
                collider,
                node,
                node.worldTransform,
                false,
                node.dirtyScale,
                isTrigger
            );
        }

        if (
            !isTrigger &&
            (node.extensions?.KHR_physics_rigid_bodies?.trigger?.mesh !== undefined ||
                node.extensions?.KHR_physics_rigid_bodies?.trigger?.shape !== undefined)
        ) {
            this.engine.updateCollider(
                state.gltf,
                node,
                node.extensions?.KHR_physics_rigid_bodies?.trigger,
                node,
                node.worldTransform,
                false,
                node.dirtyScale,
                true
            );
        }
        if (!isTrigger) {
            for (const childIndex of node.children) {
                const childNode = state.gltf.nodes[childIndex];
                if (isTrigger) {
                    collider = childNode.extensions?.KHR_physics_rigid_bodies?.trigger;
                } else {
                    collider = childNode.extensions?.KHR_physics_rigid_bodies?.collider;
                }
                PhysicsUtils.recurseCollider(
                    state.gltf,
                    childNode,
                    collider,
                    node,
                    node.dirtyScale,
                    node.dirtyScale,
                    this.engine.updateCollider.bind(this.engine)
                );
            }
        }
    }

    applyAnimations(state) {
        this.engine.updatePhysicMaterials(state.gltf);

        for (const actorNode of this.staticActors) {
            this.updateColliders(state, actorNode);
        }

        for (const actorNode of this.kinematicActors) {
            this.engine.updateMotion(actorNode);
            this.updateColliders(state, actorNode);
        }

        for (const actorNode of this.dynamicActors) {
            this.engine.updateMotion(actorNode);
            this.updateColliders(state, actorNode);
        }

        for (const node of this.independentTriggerNodes) {
            this.updateColliders(state, node, true);
        }

        for (const jointNode of this.jointNodes) {
            this.engine.updatePhysicsJoint(state, jointNode); //TODO
        }
    }

    enableDebugColliders(enable) {
        this.engine.enableDebugColliders(enable);
    }

    enableDebugJoints(enable) {
        this.engine.enableDebugJoints(enable);
    }

    getDebugLineData() {
        if (this.engine) {
            return this.engine.getDebugLineData();
        }
        return [];
    }

    applyImpulse(nodeIndex, linearImpulse, angularImpulse) {
        this.engine.applyImpulse(nodeIndex, linearImpulse, angularImpulse);
    }

    applyPointImpulse(nodeIndex, impulse, position) {
        this.engine.applyPointImpulse(nodeIndex, impulse, position);
    }

    rayCast(rayStart, rayEnd) {
        return this.engine.rayCast(rayStart, rayEnd);
    }
}

export { PhysicsController };
