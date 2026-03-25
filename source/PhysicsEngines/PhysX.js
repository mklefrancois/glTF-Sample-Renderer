import PhysX from "physx-js-webidl";
// The import is needed for rollup to include the wasm file in the build output
// eslint-disable-next-line no-unused-vars
import PhysXBinaryFile from "physx-js-webidl/physx-js-webidl.wasm";
import { gltfPhysicsMaterial } from "../gltf/rigid_bodies";
import {
    createBoxVertexData,
    createCapsuleVertexData,
    createCylinderVertexData
} from "../geometry_generator";
import { vec3, mat4, quat, mat3 } from "gl-matrix";
import { PhysicsInterface } from "./PhysicsInterface";
import { PhysicsUtils } from "../gltf/physics_utils";

class NvidiaPhysicsInterface extends PhysicsInterface {
    constructor() {
        super();
        this.PhysX = undefined;
        this.physics = undefined;
        this.defaultMaterial = undefined;
        this.tolerances = undefined;

        this.reset = false;

        // Needs to be reset for each scene
        this.scene = undefined;
        this.nodeToActor = new Map();
        this.nodeToMotion = new Map();
        this.nodeToSimplifiedJoints = new Map();
        this.shapeToNode = new Map();
        this.filterData = [];
        this.physXFilterData = [];
        this.physXMaterials = [];

        // Need for memory management
        this.convexMeshes = [];
        this.triangleMeshes = [];

        // Debug
        this.debugColliders = false;
        this.debugJoints = false;
        this.debugStateChanged = true;

        this.MAX_FLOAT = 3.4028234663852885981170418348452e38;
    }

    async initializeEngine() {
        this.PhysX = await PhysX({ locateFile: () => "./libs/physx-js-webidl.wasm" });
        const version = this.PhysX.PHYSICS_VERSION;
        console.log(
            "PhysX loaded! Version: " +
                ((version >> 24) & 0xff) +
                "." +
                ((version >> 16) & 0xff) +
                "." +
                ((version >> 8) & 0xff)
        );

        const allocator = new this.PhysX.PxDefaultAllocator();
        const errorCb = new this.PhysX.PxDefaultErrorCallback();
        const foundation = this.PhysX.CreateFoundation(version, allocator, errorCb);
        console.log("Created PxFoundation");

        this.tolerances = new this.PhysX.PxTolerancesScale();
        this.tolerances.speed = 9.81;
        this.physics = this.PhysX.CreatePhysics(version, foundation, this.tolerances);
        this.defaultMaterial = this.createPhysXMaterial(new gltfPhysicsMaterial());
        console.log("Created PxPhysics");
        return this.PhysX;
    }

    updatePhysicMaterials(gltf) {
        const materials = gltf.extensions?.KHR_physics_rigid_bodies?.physicsMaterials;
        if (materials === undefined) {
            return;
        }
        for (let i = 0; i < materials.length; i++) {
            const material = materials[i];
            if (material.isDirty()) {
                const physXMaterial = this.physXMaterials[i];
                physXMaterial.setStaticFriction(material.staticFriction);
                physXMaterial.setDynamicFriction(material.dynamicFriction);
                physXMaterial.setRestitution(material.restitution);
            }
        }
    }

    updateActorTransform(node) {
        if (node.dirtyTransform) {
            const actor = this.nodeToActor.get(node.gltfObjectIndex)?.actor;
            if (actor === undefined) {
                return;
            }
            const pxPos = new this.PhysX.PxVec3(
                node.worldTransform[12],
                node.worldTransform[13],
                node.worldTransform[14]
            );
            const pxRot = new this.PhysX.PxQuat(...node.worldQuaternion);
            const pxTransform = new this.PhysX.PxTransform(pxPos, pxRot);
            if (node?.extensions?.KHR_physics_rigid_bodies?.motion?.isKinematic) {
                actor.setKinematicTarget(pxTransform);
            } else {
                actor.setGlobalPose(pxTransform);
            }
            this.PhysX.destroy(pxPos);
            this.PhysX.destroy(pxRot);
            this.PhysX.destroy(pxTransform);
        }
    }

    calculateMassAndInertia(motion, actor) {
        const pos = new this.PhysX.PxVec3(0, 0, 0);
        if (motion.centerOfMass !== undefined) {
            pos.x = motion.centerOfMass[0];
            pos.y = motion.centerOfMass[1];
            pos.z = motion.centerOfMass[2];
        }
        const rot = new this.PhysX.PxQuat(this.PhysX.PxIDENTITYEnum.PxIdentity);
        if (motion.inertiaDiagonal !== undefined) {
            let inertia = undefined;
            if (
                motion.inertiaOrientation !== undefined &&
                !quat.exactEquals(motion.inertiaOrientation, quat.create())
            ) {
                const intertiaRotMat = mat3.create();

                const inertiaDiagonalMat = mat3.create();
                inertiaDiagonalMat[0] = motion.inertiaDiagonal[0];
                inertiaDiagonalMat[4] = motion.inertiaDiagonal[1];
                inertiaDiagonalMat[8] = motion.inertiaDiagonal[2];

                if (
                    quat.length(motion.inertiaOrientation) > 1.0e-5 ||
                    quat.length(motion.inertiaOrientation) < 1.0e-5
                ) {
                    mat3.identity(intertiaRotMat);
                    console.warn(
                        "PhysX: Invalid inertia orientation quaternion, ignoring rotation"
                    );
                } else {
                    mat3.fromQuat(intertiaRotMat, motion.inertiaOrientation);
                }

                const inertiaTensor = mat3.create();
                mat3.multiply(inertiaTensor, intertiaRotMat, inertiaDiagonalMat);

                const col0 = new this.PhysX.PxVec3(
                    inertiaTensor[0],
                    inertiaTensor[1],
                    inertiaTensor[2]
                );
                const col1 = new this.PhysX.PxVec3(
                    inertiaTensor[3],
                    inertiaTensor[4],
                    inertiaTensor[5]
                );
                const col2 = new this.PhysX.PxVec3(
                    inertiaTensor[6],
                    inertiaTensor[7],
                    inertiaTensor[8]
                );
                const pxInertiaTensor = new this.PhysX.PxMat33(col0, col1, col2);
                inertia = this.PhysX.PxMassProperties.prototype.getMassSpaceInertia(
                    pxInertiaTensor,
                    rot
                );
                this.PhysX.destroy(col0);
                this.PhysX.destroy(col1);
                this.PhysX.destroy(col2);
                this.PhysX.destroy(pxInertiaTensor);
                actor.setMassSpaceInertiaTensor(inertia);
            } else {
                inertia = new this.PhysX.PxVec3(...motion.inertiaDiagonal);
                actor.setMassSpaceInertiaTensor(inertia);
                this.PhysX.destroy(inertia);
            }
        } else {
            if (motion.mass === undefined) {
                this.PhysX.PxRigidBodyExt.prototype.updateMassAndInertia(actor, 1.0, pos);
            } else {
                this.PhysX.PxRigidBodyExt.prototype.setMassAndUpdateInertia(
                    actor,
                    motion.mass,
                    pos
                );
            }
        }

        const pose = new this.PhysX.PxTransform(pos, rot);
        actor.setCMassLocalPose(pose);
        this.PhysX.destroy(pos);
        this.PhysX.destroy(rot);
        this.PhysX.destroy(pose);
    }

    updateMotion(actorNode) {
        const motion = actorNode.extensions?.KHR_physics_rigid_bodies?.motion;
        const actor = this.nodeToActor.get(actorNode.gltfObjectIndex).actor;
        if (motion.animatedPropertyObjects.isKinematic.dirty) {
            if (motion.isKinematic) {
                const linearVelocity = actor.getLinearVelocity();
                motion.computedLinearVelocity = [
                    linearVelocity.x,
                    linearVelocity.y,
                    linearVelocity.z
                ];
                const angularVelocity = actor.getAngularVelocity();
                motion.computedAngularVelocity = [
                    angularVelocity.x,
                    angularVelocity.y,
                    angularVelocity.z
                ];
            } else {
                motion.computedLinearVelocity = undefined;
                motion.computedAngularVelocity = undefined;
            }
            actor.setRigidBodyFlag(this.PhysX.PxRigidBodyFlagEnum.eENABLE_CCD, !motion.isKinematic);
            actor.setRigidBodyFlag(this.PhysX.PxRigidBodyFlagEnum.eKINEMATIC, motion.isKinematic);
        }
        if (motion.animatedPropertyObjects.mass.dirty) {
            actor.setMass(motion.mass);
        }
        if (
            motion.animatedPropertyObjects.centerOfMass.dirty ||
            motion.animatedPropertyObjects.inertiaOrientation.dirty ||
            motion.animatedPropertyObjects.inertiaDiagonal.dirty
        ) {
            this.calculateMassAndInertia(motion, actor);
        }
        if (motion.animatedPropertyObjects.gravityFactor.dirty) {
            actor.setActorFlag(
                this.PhysX.PxActorFlagEnum.eDISABLE_GRAVITY,
                motion.gravityFactor !== 1.0
            );
        }
        if (motion.animatedPropertyObjects.linearVelocity.dirty) {
            const pxVelocity = new this.PhysX.PxVec3(...motion.linearVelocity);
            actor.setLinearVelocity(pxVelocity);
            motion.computedLinearVelocity = undefined;
        }
        if (motion.animatedPropertyObjects.angularVelocity.dirty) {
            const pxVelocity = new this.PhysX.PxVec3(...motion.angularVelocity);
            actor.setAngularVelocity(pxVelocity);
            motion.computedAngularVelocity = undefined;
        }
    }

    updateCollider(
        gltf,
        node,
        collider,
        actorNode,
        worldTransform,
        offsetChanged,
        scaleChanged,
        isTrigger
    ) {
        const result = this.nodeToActor.get(actorNode.gltfObjectIndex);
        const actor = result?.actor;
        const currentShape = result?.pxShapeMap.get(node.gltfObjectIndex);

        let currentGeometry = currentShape.getGeometry();
        const currentColliderType = currentGeometry.getType();
        const shapeIndex = collider?.geometry?.shape;
        let scale = vec3.fromValues(1, 1, 1);
        let scaleAxis = quat.create();
        if (shapeIndex !== undefined) {
            // Simple shapes need to be recreated if scale changed
            // If properties changed we also need to recreate the mesh colliders
            const dirty = gltf.extensions.KHR_implicit_shapes.shapes[shapeIndex].isDirty();
            if (
                scaleChanged &&
                !dirty &&
                currentColliderType === this.PhysX.PxGeometryTypeEnum.eCONVEXMESH
            ) {
                // Update convex mesh scale
                currentGeometry = this.PhysX.castObject(
                    currentGeometry,
                    this.PhysX.PxConvexMeshGeometry
                );
                const result = PhysicsUtils.calculateScaleAndAxis(node);
                scale = result.scale;
                scaleAxis = result.scaleAxis;
                const pxScale = new this.PhysX.PxVec3(...scale);
                const pxRotation = new this.PhysX.PxQuat(...scaleAxis);
                const meshScale = new this.PhysX.PxMeshScale(pxScale, pxRotation);
                currentGeometry.scale = meshScale;
                this.PhysX.destroy(pxScale);
                this.PhysX.destroy(pxRotation);
            } else if (dirty || scaleChanged) {
                // Recreate simple shape collider
                const newGeometry = this.generateSimpleShape(
                    gltf.extensions.KHR_implicit_shapes.shapes[shapeIndex],
                    scale,
                    scaleAxis
                );
                currentGeometry.release?.();
                if (newGeometry.getType() !== currentColliderType) {
                    // We need to recreate the shape
                    let shapeFlags = undefined;
                    if (isTrigger) {
                        shapeFlags = this.PhysX.PxShapeFlagEnum.eTRIGGER_SHAPE;
                    } else {
                        shapeFlags = this.PhysX.PxShapeFlagEnum.eSIMULATION_SHAPE;
                    }
                    if (this.debugColliders) {
                        shapeFlags |= this.PhysX.PxShapeFlagEnum.eVISUALIZATION;
                    }
                    const shape = this.createShapeFromGeometry(
                        newGeometry,
                        undefined,
                        undefined,
                        shapeFlags,
                        collider
                    );
                    result?.pxShapeMap.set(node.gltfObjectIndex, shape);
                    actor.detachShape(currentShape);
                    actor.attachShape(shape);
                    this.PhysX.destroy(currentShape);
                } else {
                    currentShape.setGeometry(newGeometry);
                }
            }
        } else if (collider?.geometry?.mesh !== undefined) {
            if (scaleChanged) {
                if (currentColliderType === this.PhysX.PxGeometryTypeEnum.eCONVEXMESH) {
                    currentGeometry = this.PhysX.castObject(
                        currentGeometry,
                        this.PhysX.PxConvexMeshGeometry
                    );
                } else if (currentColliderType === this.PhysX.PxGeometryTypeEnum.eTRIANGLEMESH) {
                    currentGeometry = this.PhysX.castObject(
                        currentGeometry,
                        this.PhysX.PxTriangleMeshGeometry
                    );
                }
                // apply scale
                const result = PhysicsUtils.calculateScaleAndAxis(node);
                scale = result.scale;
                scaleAxis = result.scaleAxis;
                const pxScale = new this.PhysX.PxVec3(...scale);
                const pxRotation = new this.PhysX.PxQuat(...scaleAxis);
                const meshScale = new this.PhysX.PxMeshScale(pxScale, pxRotation);
                currentGeometry.scale = meshScale;
                this.PhysX.destroy(pxScale);
                this.PhysX.destroy(pxRotation);
            }
        }
        if (offsetChanged) {
            // Calculate offset position
            const translation = vec3.create();
            const shapePosition = vec3.create();
            mat4.getTranslation(shapePosition, actorNode.worldTransform);
            const invertedActorRotation = quat.create();
            quat.invert(invertedActorRotation, actorNode.worldQuaternion);
            const offsetPosition = vec3.create();
            mat4.getTranslation(offsetPosition, worldTransform);
            vec3.subtract(translation, offsetPosition, shapePosition);
            vec3.transformQuat(translation, translation, invertedActorRotation);

            // Calculate offset rotation
            const rotation = quat.create();
            quat.multiply(rotation, invertedActorRotation, node.worldQuaternion);

            const PxPos = new this.PhysX.PxVec3(...translation);
            const PxRotation = new this.PhysX.PxQuat(...rotation);
            const pose = new this.PhysX.PxTransform(PxPos, PxRotation);
            currentShape.setLocalPose(pose);
        }
    }

    updatePhysicsJoint(state, jointNode) {
        const pxJoints = this.nodeToSimplifiedJoints.get(jointNode.gltfObjectIndex);
        if (pxJoints === undefined) {
            return;
        }
        const gltfJoint =
            state.gltf.extensions.KHR_physics_rigid_bodies.physicsJoints[
                jointNode.extensions.KHR_physics_rigid_bodies.joint.joint
            ];
        const simplifiedJoints = gltfJoint.simplifiedPhysicsJoints;
        if (simplifiedJoints.length !== pxJoints.length) {
            console.warn(
                "Number of simplified joints does not match number of PhysX joints. Skipping joint update."
            );
            return;
        }
        for (let i = 0; i < simplifiedJoints.length; i++) {
            const pxJoint = pxJoints[i];
            const simplifiedJoint = simplifiedJoints[i];
            if (
                jointNode.extensions.KHR_physics_rigid_bodies.joint.animatedPropertyObjects
                    .enableCollision.dirty
            ) {
                pxJoint.setConstraintFlag(
                    this.PhysX.PxConstraintFlagEnum.eCOLLISION_ENABLED,
                    jointNode.extensions.KHR_physics_rigid_bodies.joint.enableCollision
                );
            }
            for (const limit of simplifiedJoint.limits) {
                if (
                    limit.animatedPropertyObjects.min.dirty ||
                    limit.animatedPropertyObjects.max.dirty ||
                    limit.animatedPropertyObjects.stiffness.dirty ||
                    limit.animatedPropertyObjects.damping.dirty
                ) {
                    this._setLimitValues(pxJoint, simplifiedJoint, limit);
                }
            }

            if (
                simplifiedJoint.twistLimit &&
                (simplifiedJoint.twistLimit.animatedPropertyObjects.min.dirty ||
                    simplifiedJoint.twistLimit.animatedPropertyObjects.max.dirty ||
                    simplifiedJoint.twistLimit.animatedPropertyObjects.stiffness.dirty ||
                    simplifiedJoint.twistLimit.animatedPropertyObjects.damping.dirty)
            ) {
                this._setTwistLimitValues(pxJoint, simplifiedJoint);
            }

            if (
                (simplifiedJoint.swingLimit1 &&
                    (simplifiedJoint.swingLimit1.animatedPropertyObjects.min.dirty ||
                        simplifiedJoint.swingLimit1.animatedPropertyObjects.max.dirty ||
                        simplifiedJoint.swingLimit1.animatedPropertyObjects.stiffness.dirty ||
                        simplifiedJoint.swingLimit1.animatedPropertyObjects.damping.dirty)) ||
                (simplifiedJoint.swingLimit2 &&
                    (simplifiedJoint.swingLimit2.animatedPropertyObjects.min.dirty ||
                        simplifiedJoint.swingLimit2.animatedPropertyObjects.max.dirty ||
                        simplifiedJoint.swingLimit2.animatedPropertyObjects.stiffness.dirty ||
                        simplifiedJoint.swingLimit2.animatedPropertyObjects.damping.dirty))
            ) {
                this._setSwingLimitValues(pxJoint, simplifiedJoint);
            }

            let positionTargetDirty = false;
            let velocityTargetDirty = false;
            const linearVelocityTarget = new this.PhysX.PxVec3(0, 0, 0);
            const angularVelocityTarget = new this.PhysX.PxVec3(0, 0, 0);
            pxJoint.getDriveVelocity(linearVelocityTarget, angularVelocityTarget);
            for (const drive of simplifiedJoint.drives) {
                if (
                    drive.animatedPropertyObjects.stiffness.dirty ||
                    drive.animatedPropertyObjects.damping.dirty ||
                    drive.animatedPropertyObjects.maxForce.dirty
                ) {
                    this._setDriveValues(pxJoint, simplifiedJoint, drive);
                }
                if (drive.animatedPropertyObjects.velocityTarget.dirty) {
                    this._getDriveVelocityTarget(
                        simplifiedJoint,
                        drive,
                        linearVelocityTarget,
                        angularVelocityTarget
                    );
                    velocityTargetDirty = true;
                }
                if (drive.animatedPropertyObjects.positionTarget.dirty) {
                    positionTargetDirty = true;
                }
            }

            if (positionTargetDirty) {
                this._setDrivePositionTarget(pxJoint, simplifiedJoint);
            }
            if (velocityTargetDirty) {
                pxJoint.setDriveVelocity(linearVelocityTarget, angularVelocityTarget);
            }
        }
    }

    mapCombineMode(mode) {
        switch (mode) {
            case "average":
                return this.PhysX.PxCombineModeEnum.eAVERAGE;
            case "minimum":
                return this.PhysX.PxCombineModeEnum.eMIN;
            case "maximum":
                return this.PhysX.PxCombineModeEnum.eMAX;
            case "multiply":
                return this.PhysX.PxCombineModeEnum.eMULTIPLY;
        }
    }

    // Either create a box or update an existing one. Returns only newly created geometry
    generateBox(x, y, z, scale, scaleAxis, reference) {
        let referenceType = undefined;
        if (reference !== undefined) {
            referenceType = reference.getType();
        }
        if (
            scale.every((value) => value === scale[0]) === false &&
            quat.equals(scaleAxis, quat.create()) === false
        ) {
            const data = createBoxVertexData(x, y, z);
            return this.createConvexPxMesh(data.vertices, scale, scaleAxis);
        }
        let geometry = undefined;
        if (referenceType === this.PhysX.PxGeometryTypeEnum.eBOX) {
            const halfExtents = new this.PhysX.PxVec3(
                (x / 2) * scale[0],
                (y / 2) * scale[1],
                (z / 2) * scale[2]
            );
            reference.halfExtents = halfExtents;
            this.PhysX.destroy(halfExtents);
        } else {
            geometry = new this.PhysX.PxBoxGeometry(
                (x / 2) * scale[0],
                (y / 2) * scale[1],
                (z / 2) * scale[2]
            );
        }

        return geometry;
    }

    generateCapsule(height, radiusTop, radiusBottom, scale, scaleAxis, _reference) {
        const data = createCapsuleVertexData(radiusTop, radiusBottom, height);
        return this.createConvexPxMesh(data.vertices, scale, scaleAxis);
    }

    generateCylinder(height, radiusTop, radiusBottom, scale, scaleAxis, _reference) {
        if (
            (quat.equals(scaleAxis, quat.create()) === false &&
                scale.every((value) => value === scale[0]) === false) ||
            radiusTop !== radiusBottom ||
            scale[0] !== scale[2]
        ) {
            const data = createCylinderVertexData(radiusTop, radiusBottom, height);
            return this.createConvexPxMesh(data.vertices, scale, scaleAxis);
        }
        height *= scale[1];
        radiusTop *= scale[0];
        radiusBottom *= scale[0];
        const data = createCylinderVertexData(radiusTop, radiusBottom, height);
        return this.createConvexPxMesh(data.vertices);
    }

    generateSphere(radius, scale, scaleAxis, reference) {
        let referenceType = undefined;
        if (reference !== undefined) {
            referenceType = reference.getType();
        }
        if (scale.every((value) => value === scale[0]) === false) {
            const data = createCapsuleVertexData(radius, radius, 0);
            return this.createConvexPxMesh(data.vertices, scale, scaleAxis);
        } else {
            radius *= scale[0];
        }
        if (referenceType === this.PhysX.PxGeometryTypeEnum.eSPHERE) {
            reference.radius = radius;
            return undefined;
        }
        const geometry = new this.PhysX.PxSphereGeometry(radius);
        return geometry;
    }

    generatePlane(width, height, doubleSided, scale, scaleAxis, reference) {
        if (reference !== undefined) {
            //TODO handle update
            return undefined;
        }
        const geometry = new this.PhysX.PxPlaneGeometry();
        return geometry;
    }

    createConvexPxMesh(vertices, scale = vec3.fromValues(1, 1, 1), scaleAxis = quat.create()) {
        const malloc = (f, q) => {
            const nDataBytes = f.length * f.BYTES_PER_ELEMENT;
            if (q === undefined) q = this.PhysX._webidl_malloc(nDataBytes);
            let dataHeap = new Uint8Array(this.PhysX.HEAPU8.buffer, q, nDataBytes);
            dataHeap.set(new Uint8Array(f.buffer));
            return q;
        };
        const des = new this.PhysX.PxConvexMeshDesc();
        des.points.stride = vertices.BYTES_PER_ELEMENT * 3;
        des.points.count = vertices.length / 3;
        des.points.data = malloc(vertices);

        let flag = 0;
        flag |= this.PhysX.PxConvexFlagEnum.eCOMPUTE_CONVEX;
        flag |= this.PhysX.PxConvexFlagEnum.eSHIFT_VERTICES;
        //flag |= this.PhysX.PxConvexFlagEnum.eDISABLE_MESH_VALIDATION;
        const pxflags = new this.PhysX.PxConvexFlags(flag);
        des.flags = pxflags;
        const cookingParams = new this.PhysX.PxCookingParams(this.tolerances);
        cookingParams.planeTolerance = 0.0007; //Default
        const tri = this.PhysX.CreateConvexMesh(cookingParams, des);
        this.convexMeshes.push(tri);

        const PxScale = new this.PhysX.PxVec3(scale[0], scale[1], scale[2]);
        const PxQuat = new this.PhysX.PxQuat(...scaleAxis);
        const ms = new this.PhysX.PxMeshScale(PxScale, PxQuat);
        const f = new this.PhysX.PxConvexMeshGeometryFlags();
        const geometry = new this.PhysX.PxConvexMeshGeometry(tri, ms, f);
        this.PhysX.destroy(PxScale);
        this.PhysX.destroy(PxQuat);
        this.PhysX.destroy(ms);
        this.PhysX.destroy(pxflags);
        this.PhysX.destroy(cookingParams);
        this.PhysX.destroy(des);
        return geometry;
    }

    collectVerticesAndIndicesFromMesh(gltf, mesh, computeIndices = true) {
        let positionDataArray = [];
        let positionCount = 0;
        let indexDataArray = [];
        let indexCount = 0;

        for (const primitive of mesh.primitives) {
            const positionAccessor = gltf.accessors[primitive.attributes.POSITION];
            const positionData = positionAccessor.getNormalizedDeinterlacedView(gltf);

            if (primitive.targets !== undefined) {
                let morphWeights = mesh.weights;
                if (morphWeights !== undefined) {
                    // Calculate morphed vertex positions on CPU
                    const morphPositionData = [];
                    for (const target of primitive.targets) {
                        if (target.POSITION !== undefined) {
                            const morphAccessor = gltf.accessors[target.POSITION];
                            morphPositionData.push(
                                morphAccessor.getNormalizedDeinterlacedView(gltf)
                            );
                        } else {
                            morphPositionData.push(undefined);
                        }
                    }
                    for (let i = 0; i < positionData.length / 3; i++) {
                        for (let j = 0; j < morphWeights.length; j++) {
                            const morphData = morphPositionData[j];
                            if (morphWeights[j] === 0 || morphData === undefined) {
                                continue;
                            }
                            positionData[i * 3] += morphData[i * 3] * morphWeights[j];
                            positionData[i * 3 + 1] += morphData[i * 3 + 1] * morphWeights[j];
                            positionData[i * 3 + 2] += morphData[i * 3 + 2] * morphWeights[j];
                        }
                    }
                }
            }

            positionDataArray.push(positionData);
            positionCount += positionAccessor.count;
            if (computeIndices) {
                let indexData = undefined;
                if (primitive.indices !== undefined) {
                    const indexAccessor = gltf.accessors[primitive.indices];
                    indexData = indexAccessor.getNormalizedDeinterlacedView(gltf);
                } else {
                    const array = Array.from(Array(positionAccessor.count).keys());
                    indexData = new Uint32Array(array);
                }
                if (primitive.mode === 5) {
                    indexData = PhysicsUtils.convertTriangleStripToTriangles(indexData);
                } else if (primitive.mode === 6) {
                    indexData = PhysicsUtils.convertTriangleFanToTriangles(indexData);
                } else if (primitive.mode !== undefined && primitive.mode !== 4) {
                    console.warn(
                        "Unsupported primitive mode for physics mesh collider creation: " +
                            primitive.mode
                    );
                }
                indexDataArray.push(indexData);
                indexCount += indexData.length;
            }
        }

        const positionData = new Float32Array(positionCount * 3);
        const indexData = new Uint32Array(indexCount);
        let offset = 0;
        for (const positionChunk of positionDataArray) {
            positionData.set(positionChunk, offset);
            offset += positionChunk.length;
        }
        offset = 0;
        for (const indexChunk of indexDataArray) {
            indexData.set(indexChunk, offset);
            offset += indexChunk.length;
        }
        return { vertices: positionData, indices: indexData };
    }

    createConvexMesh(gltf, mesh, scale = vec3.fromValues(1, 1, 1), scaleAxis = quat.create()) {
        const result = this.collectVerticesAndIndicesFromMesh(gltf, mesh, false);
        return this.createConvexPxMesh(result.vertices, scale, scaleAxis);
    }

    createPxMesh(gltf, mesh, scale = vec3.fromValues(1, 1, 1), scaleAxis = quat.create()) {
        const { vertices, indices } = this.collectVerticesAndIndicesFromMesh(gltf, mesh, true);
        const malloc = (f, q) => {
            const nDataBytes = f.length * f.BYTES_PER_ELEMENT;
            if (q === undefined) q = this.PhysX._webidl_malloc(nDataBytes);
            let dataHeap = new Uint8Array(this.PhysX.HEAPU8.buffer, q, nDataBytes);
            dataHeap.set(new Uint8Array(f.buffer));
            return q;
        };
        const des = new this.PhysX.PxTriangleMeshDesc();
        des.points.stride = vertices.BYTES_PER_ELEMENT * 3;
        des.points.count = vertices.length / 3;
        des.points.data = malloc(vertices);

        des.triangles.stride = indices.BYTES_PER_ELEMENT * 3;
        des.triangles.count = indices.length / 3;
        des.triangles.data = malloc(indices);

        const cookingParams = new this.PhysX.PxCookingParams(this.tolerances);
        const tri = this.PhysX.CreateTriangleMesh(cookingParams, des);
        this.triangleMeshes.push(tri);

        const PxScale = new this.PhysX.PxVec3(1, 1, 1);
        const PxQuat = new this.PhysX.PxQuat(0, 0, 0, 1);

        if (scale !== undefined) {
            PxScale.x = scale[0];
            PxScale.y = scale[1];
            PxScale.z = scale[2];
        }
        if (scaleAxis !== undefined) {
            PxQuat.x = scaleAxis[0];
            PxQuat.y = scaleAxis[1];
            PxQuat.z = scaleAxis[2];
            PxQuat.w = scaleAxis[3];
        }
        const ms = new this.PhysX.PxMeshScale(PxScale, PxQuat);
        const f = new this.PhysX.PxMeshGeometryFlags();
        const geometry = new this.PhysX.PxTriangleMeshGeometry(tri, ms, f);
        this.PhysX.destroy(PxScale);
        this.PhysX.destroy(PxQuat);
        this.PhysX.destroy(ms);
        this.PhysX.destroy(cookingParams);
        this.PhysX.destroy(des);
        return geometry;
    }

    collidesWith(filterA, filterB) {
        if (filterB.collideWithSystems.length > 0) {
            for (const system of filterB.collideWithSystems) {
                if (filterA.collisionSystems.includes(system)) {
                    return true;
                }
            }
            return false;
        } else if (filterB.notCollideWithSystems.length > 0) {
            for (const system of filterB.notCollideWithSystems) {
                if (filterA.collisionSystems.includes(system)) {
                    return false;
                }
            }
            return true;
        }
        return true;
    }

    computeFilterData(gltf) {
        // Default filter is sign bit
        const filters = gltf.extensions?.KHR_physics_rigid_bodies?.collisionFilters;
        this.filterData = new Array(32).fill(0);
        this.filterData[31] = Math.pow(2, 32) - 1; // Default filter with all bits set
        let filterCount = filters?.length ?? 0;
        if (filterCount > 31) {
            filterCount = 31;
            console.warn(
                "PhysX supports a maximum of 31 collision filters. Additional filters will be ignored."
            );
        }

        for (let i = 0; i < filterCount; i++) {
            let bitMask = 0;
            for (let j = 0; j < filterCount; j++) {
                if (
                    this.collidesWith(filters[i], filters[j]) &&
                    this.collidesWith(filters[j], filters[i])
                ) {
                    bitMask |= 1 << j;
                }
            }
            this.filterData[i] = bitMask;
        }
    }

    createPhysXMaterial(gltfPhysicsMaterial) {
        if (gltfPhysicsMaterial === undefined) {
            return this.defaultMaterial;
        }

        const physxMaterial = this.physics.createMaterial(
            gltfPhysicsMaterial.staticFriction,
            gltfPhysicsMaterial.dynamicFriction,
            gltfPhysicsMaterial.restitution
        );
        if (gltfPhysicsMaterial.frictionCombine !== undefined) {
            physxMaterial.setFrictionCombineMode(
                this.mapCombineMode(gltfPhysicsMaterial.frictionCombine)
            );
        }
        if (gltfPhysicsMaterial.restitutionCombine !== undefined) {
            physxMaterial.setRestitutionCombineMode(
                this.mapCombineMode(gltfPhysicsMaterial.restitutionCombine)
            );
        }
        return physxMaterial;
    }

    createPhysXCollisionFilter(collisionFilter, additionalFlags = 0) {
        let word0 = null;
        let word1 = null;
        if (collisionFilter !== undefined && collisionFilter < this.filterData.length - 1) {
            word0 = 1 << collisionFilter;
            word1 = this.filterData[collisionFilter];
        } else {
            // Default filter id is signed bit and all bits set to collide with everything
            word0 = Math.pow(2, 31);
            word1 = Math.pow(2, 32) - 1;
        }

        additionalFlags |= this.PhysX.PxPairFlagEnum.eDETECT_DISCRETE_CONTACT;
        additionalFlags |= this.PhysX.PxPairFlagEnum.eDETECT_CCD_CONTACT;

        return new this.PhysX.PxFilterData(word0, word1, additionalFlags, 0);
    }

    createShapeFromGeometry(geometry, physXMaterial, physXFilterData, shapeFlags, glTFCollider) {
        if (physXMaterial === undefined) {
            if (glTFCollider?.physicsMaterial !== undefined) {
                physXMaterial = this.physXMaterials[glTFCollider.physicsMaterial];
            } else {
                physXMaterial = this.defaultMaterial;
            }
        }
        const shape = this.physics.createShape(geometry, physXMaterial, true, shapeFlags);

        if (physXFilterData === undefined) {
            physXFilterData =
                this.physXFilterData[
                    glTFCollider?.collisionFilter ?? this.physXFilterData.length - 1
                ];
        }

        shape.setSimulationFilterData(physXFilterData);

        return shape;
    }

    createShape(
        gltf,
        node,
        collider,
        shapeFlags,
        physXMaterial,
        physXFilterData,
        convexHull,
        scale = vec3.fromValues(1, 1, 1),
        scaleAxis = quat.create()
    ) {
        let geometry = undefined;
        if (collider?.geometry?.shape !== undefined) {
            if (scale[0] !== 1 || scale[1] !== 1 || scale[2] !== 1) {
                const simpleShape =
                    gltf.extensions.KHR_implicit_shapes.shapes[collider.geometry.shape];
                geometry = this.generateSimpleShape(simpleShape, scale, scaleAxis);
            } else {
                geometry = this.simpleShapes[collider.geometry.shape];
            }
        } else if (collider?.geometry?.mesh !== undefined) {
            const mesh = gltf.meshes[collider.geometry.mesh];
            if (convexHull === true) {
                geometry = this.createConvexMesh(gltf, mesh, scale, scaleAxis);
            } else {
                geometry = this.createPxMesh(gltf, mesh, scale, scaleAxis);
            }
        }

        if (geometry === undefined) {
            return undefined;
        }

        const shape = this.createShapeFromGeometry(
            geometry,
            physXMaterial,
            physXFilterData,
            shapeFlags,
            collider
        );

        this.shapeToNode.set(shape.ptr, node.gltfObjectIndex);
        return shape;
    }

    createActor(gltf, node, shapeFlags, triggerFlags, type, noMeshShapes = false) {
        const worldTransform = node.worldTransform;
        const translation = vec3.create();
        mat4.getTranslation(translation, worldTransform);
        const pos = new this.PhysX.PxVec3(...translation);
        const rotation = new this.PhysX.PxQuat(...node.worldQuaternion);
        const pose = new this.PhysX.PxTransform(pos, rotation);
        let actor = null;
        const pxShapeMap = new Map();
        if (type === "static" || type === "trigger") {
            actor = this.physics.createRigidStatic(pose);
        } else {
            actor = this.physics.createRigidDynamic(pose);
            if (type === "kinematic") {
                actor.setRigidBodyFlag(this.PhysX.PxRigidBodyFlagEnum.eKINEMATIC, true);
            }
            actor.setRigidBodyFlag(
                this.PhysX.PxRigidBodyFlagEnum.eENABLE_CCD,
                type !== "kinematic"
            );
            const motion = node.extensions?.KHR_physics_rigid_bodies?.motion;
            if (motion) {
                const gltfAngularVelocity = motion?.angularVelocity;
                const angularVelocity = new this.PhysX.PxVec3(...gltfAngularVelocity);
                actor.setAngularVelocity(angularVelocity, true);
                this.PhysX.destroy(angularVelocity);

                const gltfLinearVelocity = motion?.linearVelocity;
                const linearVelocity = new this.PhysX.PxVec3(...gltfLinearVelocity);
                actor.setLinearVelocity(linearVelocity, true);
                this.PhysX.destroy(linearVelocity);

                if (motion.mass !== undefined) {
                    actor.setMass(motion.mass);
                }

                this.calculateMassAndInertia(motion, actor);

                if (motion.gravityFactor !== 1.0) {
                    actor.setActorFlag(this.PhysX.PxActorFlagEnum.eDISABLE_GRAVITY, true);
                }
            }
        }

        const createAndAddShape = (
            gltf,
            node,
            collider,
            actorNode,
            worldTransform,
            offsetChanged,
            scaleChanged,
            isTrigger,
            noMeshShapes,
            shapeFlags,
            triggerFlags
        ) => {
            // Calculate offset position
            const translation = vec3.create();
            const shapePosition = vec3.create();
            mat4.getTranslation(shapePosition, actorNode.worldTransform);
            const invertedActorRotation = quat.create();
            quat.invert(invertedActorRotation, actorNode.worldQuaternion);
            const offsetPosition = vec3.create();
            mat4.getTranslation(offsetPosition, worldTransform);
            vec3.subtract(translation, offsetPosition, shapePosition);
            vec3.transformQuat(translation, translation, invertedActorRotation);

            // Calculate offset rotation
            const rotation = quat.create();
            quat.multiply(rotation, invertedActorRotation, node.worldQuaternion);

            // Calculate scale and scaleAxis
            const { scale, scaleAxis } = PhysicsUtils.calculateScaleAndAxis(node);

            const materialIndex = collider?.physicsMaterial;
            const material =
                materialIndex !== undefined
                    ? this.physXMaterials[materialIndex]
                    : this.defaultMaterial;

            const physXFilterData =
                collider?.collisionFilter !== undefined
                    ? this.physXFilterData[collider.collisionFilter]
                    : this.physXFilterData[this.physXFilterData.length - 1];

            const shape = this.createShape(
                gltf,
                node,
                collider,
                isTrigger ? triggerFlags : shapeFlags,
                material,
                physXFilterData,
                noMeshShapes || collider?.geometry?.convexHull === true,
                scale,
                scaleAxis
            );

            if (shape !== undefined) {
                const PxPos = new this.PhysX.PxVec3(...translation);
                const PxRotation = new this.PhysX.PxQuat(...rotation);
                const pose = new this.PhysX.PxTransform(PxPos, PxRotation);
                shape.setLocalPose(pose);

                actor.attachShape(shape);
                pxShapeMap.set(node.gltfObjectIndex, shape);
                this.PhysX.destroy(PxPos);
                this.PhysX.destroy(PxRotation);
                this.PhysX.destroy(pose);
            }
        };

        // If a node contains trigger and collider combine them

        let collider = undefined;
        if (type !== "trigger") {
            collider = node.extensions?.KHR_physics_rigid_bodies?.collider;
            createAndAddShape(
                gltf,
                node,
                collider,
                node,
                worldTransform,
                undefined,
                undefined,
                false,
                noMeshShapes,
                shapeFlags,
                triggerFlags
            );
            collider = node.extensions?.KHR_physics_rigid_bodies?.trigger;
            if (collider !== undefined) {
                createAndAddShape(
                    gltf,
                    node,
                    collider,
                    node,
                    worldTransform,
                    undefined,
                    undefined,
                    true,
                    true,
                    shapeFlags,
                    triggerFlags
                );
            }
        } else {
            collider = node.extensions?.KHR_physics_rigid_bodies?.trigger;
            createAndAddShape(
                gltf,
                node,
                collider,
                node,
                worldTransform,
                undefined,
                undefined,
                true,
                true,
                shapeFlags,
                triggerFlags
            );
        }

        if (type !== "trigger") {
            for (const childIndex of node.children) {
                const childNode = gltf.nodes[childIndex];
                PhysicsUtils.recurseCollider(
                    gltf,
                    childNode,
                    undefined,
                    node,
                    node.dirtyScale,
                    node.dirtyScale,
                    createAndAddShape,
                    [noMeshShapes, shapeFlags, triggerFlags]
                );
            }
        }

        this.PhysX.destroy(pos);
        this.PhysX.destroy(rotation);
        this.PhysX.destroy(pose);

        this.scene.addActor(actor);

        this.nodeToActor.set(node.gltfObjectIndex, { actor, pxShapeMap: pxShapeMap });
    }

    computeJointOffsetAndActor(node, referencedJoint) {
        let currentNode = node;
        while (currentNode !== undefined) {
            if (this.nodeToActor.has(currentNode.gltfObjectIndex)) {
                break;
            }
            currentNode = currentNode.parentNode;
        }

        const nodeWorldRot = node.worldQuaternion;
        const localPhysXRot = referencedJoint?.localRotation;
        if (localPhysXRot !== undefined) {
            quat.multiply(nodeWorldRot, node.worldQuaternion, localPhysXRot);
        }

        if (currentNode === undefined) {
            const pos = vec3.create();
            mat4.getTranslation(pos, node.worldTransform);

            return { actor: undefined, offsetPosition: pos, offsetRotation: nodeWorldRot };
        }
        const actor = this.nodeToActor.get(currentNode.gltfObjectIndex)?.actor;
        const inverseActorRotation = quat.create();
        quat.invert(inverseActorRotation, currentNode.worldQuaternion);
        const offsetRotation = quat.create();
        quat.multiply(offsetRotation, inverseActorRotation, nodeWorldRot);

        const actorPosition = vec3.create();
        mat4.getTranslation(actorPosition, currentNode.worldTransform);
        const nodePosition = vec3.create();
        mat4.getTranslation(nodePosition, node.worldTransform);
        const offsetPosition = vec3.create();
        vec3.subtract(offsetPosition, nodePosition, actorPosition);
        vec3.transformQuat(offsetPosition, offsetPosition, inverseActorRotation);

        return { actor: actor, offsetPosition: offsetPosition, offsetRotation: offsetRotation };
    }

    convertAxisIndexToEnum(axisIndex, type) {
        if (type === "linear") {
            switch (axisIndex) {
                case 0:
                    return this.PhysX.PxD6AxisEnum.eX;
                case 1:
                    return this.PhysX.PxD6AxisEnum.eY;
                case 2:
                    return this.PhysX.PxD6AxisEnum.eZ;
            }
        } else if (type === "angular") {
            switch (axisIndex) {
                case 0:
                    return this.PhysX.PxD6AxisEnum.eTWIST;
                case 1:
                    return this.PhysX.PxD6AxisEnum.eSWING1;
                case 2:
                    return this.PhysX.PxD6AxisEnum.eSWING2;
            }
        }
        return null;
    }

    convertAxisIndexToAngularDriveEnum(axisIndex) {
        switch (axisIndex) {
            case 0:
                return this.PhysX.PxD6DriveEnum.eTWIST;
            case 1:
                return 6; // Currently not exposed via bindings
            case 2:
                return 7; // Currently not exposed via bindings
        }
        return null;
    }

    validateSwingLimits(joint) {
        // Check if swing limits are symmetric (cone) or asymmetric (pyramid)
        if (joint.swingLimit1 && joint.swingLimit2) {
            const limit1 = joint.swingLimit1;
            const limit2 = joint.swingLimit2;

            const isSymmetric1 =
                Math.abs(limit1.min + limit1.max) < 1e-6 || limit1.min === undefined; // Centered around 0
            const isSymmetric2 =
                Math.abs(limit2.min + limit2.max) < 1e-6 || limit2.min === undefined;

            // Return if this is a cone limit (symmetric and same range) vs pyramid limit
            return isSymmetric1 && isSymmetric2;
        }
        return false;
    }

    createJoint(gltf, node) {
        const joint = node.extensions?.KHR_physics_rigid_bodies?.joint;
        const referencedJoint =
            gltf.extensions?.KHR_physics_rigid_bodies?.physicsJoints[joint.joint];

        if (referencedJoint === undefined) {
            console.error("Referenced joint not found:", joint.joint);
            return;
        }
        const simplifiedJoints = [];
        for (const simplifiedJoint of referencedJoint.simplifiedPhysicsJoints) {
            const physxJoint = this.createSimplifiedJoint(gltf, node, joint, simplifiedJoint);
            simplifiedJoints.push(physxJoint);
        }
        this.nodeToSimplifiedJoints.set(node.gltfObjectIndex, simplifiedJoints);
    }

    _setLimitValues(physxJoint, simplifiedJoint, limit) {
        const lock = limit.min === 0 && limit.max === 0;
        const spring = new this.PhysX.PxSpring(limit.stiffness ?? 0, limit.damping);
        const isDistanceLimit =
            limit.linearAxes &&
            limit.linearAxes.length === 3 &&
            (limit.min === undefined || limit.min === 0) &&
            limit.max !== 0;
        if (limit.linearAxes && limit.linearAxes.length > 0 && !isDistanceLimit) {
            const linearLimitPair = new this.PhysX.PxJointLinearLimitPair(
                limit.min ?? -this.MAX_FLOAT,
                limit.max ?? this.MAX_FLOAT,
                spring
            );
            for (const axis of limit.linearAxes) {
                const result = simplifiedJoint.getRotatedAxisAndSign(axis);
                const physxAxis = this.convertAxisIndexToEnum(result.axis, "linear");
                physxJoint.setMotion(
                    physxAxis,
                    lock ? this.PhysX.PxD6MotionEnum.eLOCKED : this.PhysX.PxD6MotionEnum.eLIMITED
                );
                if (!lock) {
                    physxJoint.setLinearLimit(physxAxis, linearLimitPair);
                }
            }
            this.PhysX.destroy(linearLimitPair);
        }
        if (isDistanceLimit) {
            const linearLimit = new this.PhysX.PxJointLinearLimit(
                limit.max ?? this.MAX_FLOAT,
                spring
            );
            physxJoint.setMotion(this.PhysX.PxD6AxisEnum.eX, this.PhysX.PxD6MotionEnum.eLIMITED);
            physxJoint.setMotion(this.PhysX.PxD6AxisEnum.eY, this.PhysX.PxD6MotionEnum.eLIMITED);
            physxJoint.setMotion(this.PhysX.PxD6AxisEnum.eZ, this.PhysX.PxD6MotionEnum.eLIMITED);
            physxJoint.setDistanceLimit(linearLimit);
            this.PhysX.destroy(linearLimit);
        }
        if (limit.angularAxes && limit.angularAxes.length > 0) {
            for (const axis of limit.angularAxes) {
                const result = simplifiedJoint.getRotatedAxisAndSign(axis);
                const physxAxis = this.convertAxisIndexToEnum(result.axis, "angular");
                physxJoint.setMotion(
                    physxAxis,
                    lock ? this.PhysX.PxD6MotionEnum.eLOCKED : this.PhysX.PxD6MotionEnum.eLIMITED
                );
            }
        }
        this.PhysX.destroy(spring);
    }

    _setTwistLimitValues(physxJoint, simplifiedJoint) {
        if (simplifiedJoint.twistLimit !== undefined) {
            if (!(simplifiedJoint.twistLimit.min === 0 && simplifiedJoint.twistLimit.max === 0)) {
                const limitPair = new this.PhysX.PxJointAngularLimitPair(
                    simplifiedJoint.twistLimit.min ?? -Math.PI,
                    simplifiedJoint.twistLimit.max ?? Math.PI,
                    new this.PhysX.PxSpring(
                        simplifiedJoint.twistLimit.stiffness ?? 0,
                        simplifiedJoint.twistLimit.damping
                    )
                );
                physxJoint.setTwistLimit(limitPair);
                this.PhysX.destroy(limitPair);
            }
        }
    }

    _setSwingLimitValues(physxJoint, simplifiedJoint) {
        if (
            simplifiedJoint.swingLimit1 !== undefined &&
            simplifiedJoint.swingLimit2 !== undefined
        ) {
            if (
                simplifiedJoint.swingLimit1.stiffness !== simplifiedJoint.swingLimit2.stiffness ||
                simplifiedJoint.swingLimit1.damping !== simplifiedJoint.swingLimit2.damping
            ) {
                console.warn(
                    "PhysX does not support different stiffness/damping for swing limits."
                );
            } else {
                const spring = new this.PhysX.PxSpring(
                    simplifiedJoint.swingLimit1.stiffness ?? 0,
                    simplifiedJoint.swingLimit1.damping
                );
                let yMin = -Math.PI / 2;
                let yMax = Math.PI / 2;
                let zMin = -Math.PI / 2;
                let zMax = Math.PI / 2;
                if (simplifiedJoint.swingLimit1.min !== undefined) {
                    yMin = simplifiedJoint.swingLimit1.min;
                }
                if (simplifiedJoint.swingLimit1.max !== undefined) {
                    yMax = simplifiedJoint.swingLimit1.max;
                }
                if (simplifiedJoint.swingLimit2.min !== undefined) {
                    zMin = simplifiedJoint.swingLimit2.min;
                }
                if (simplifiedJoint.swingLimit2.max !== undefined) {
                    zMax = simplifiedJoint.swingLimit2.max;
                }

                const isSymmetric = this.validateSwingLimits(simplifiedJoint);
                if (yMin === 0 && yMax === 0 && zMin === 0 && zMax === 0) {
                    // Fixed limit is already set
                } else if (isSymmetric) {
                    const swing1Angle = Math.max(Math.abs(yMin), Math.abs(yMax));
                    const swing2Angle = Math.max(Math.abs(zMin), Math.abs(zMax));
                    const jointLimitCone = new this.PhysX.PxJointLimitCone(
                        swing1Angle,
                        swing2Angle,
                        spring
                    );
                    physxJoint.setSwingLimit(jointLimitCone);
                    this.PhysX.destroy(jointLimitCone);
                } else {
                    const jointLimitCone = new this.PhysX.PxJointLimitPyramid(
                        yMin,
                        yMax,
                        zMin,
                        zMax,
                        spring
                    );
                    physxJoint.setPyramidSwingLimit(jointLimitCone);
                    this.PhysX.destroy(jointLimitCone);
                }
                this.PhysX.destroy(spring);
            }
        } else if (
            simplifiedJoint.swingLimit1 !== undefined ||
            simplifiedJoint.swingLimit2 !== undefined
        ) {
            const singleLimit = simplifiedJoint.swingLimit1 ?? simplifiedJoint.swingLimit2;
            if (singleLimit.min === 0 && singleLimit.max === 0) {
                // Fixed limit is already set
            } else if (singleLimit.min && -1 * singleLimit.min !== singleLimit.max) {
                console.warn(
                    "PhysX requires symmetric limits for swing limits in single axis mode."
                );
            } else {
                const spring = new this.PhysX.PxSpring(
                    singleLimit.stiffness ?? 0,
                    singleLimit.damping
                );
                const maxY = simplifiedJoint.swingLimit1?.max ?? Math.PI;
                const maxZ = simplifiedJoint.swingLimit2?.max ?? Math.PI;
                const jointLimitCone = new this.PhysX.PxJointLimitCone(maxY, maxZ, spring);
                physxJoint.setSwingLimit(jointLimitCone);
                this.PhysX.destroy(spring);
                this.PhysX.destroy(jointLimitCone);
            }
        }
    }

    _setDriveValues(physxJoint, simplifiedJoint, drive) {
        const physxDrive = new this.PhysX.PxD6JointDrive(
            drive.stiffness,
            drive.damping,
            drive.maxForce ?? this.MAX_FLOAT,
            drive.mode === "acceleration"
        );
        const result = simplifiedJoint.getRotatedAxisAndSign(drive.axis);
        if (drive.type === "linear") {
            const axis = this.convertAxisIndexToEnum(result.axis, "linear");
            physxJoint.setDrive(axis, physxDrive);
        } else if (drive.type === "angular") {
            const axis = this.convertAxisIndexToAngularDriveEnum(result.axis);
            physxJoint.setDrive(axis, physxDrive);
        }
        this.PhysX.destroy(physxDrive);
    }

    _getDriveVelocityTarget(simplifiedJoint, drive, linearVelocityTarget, angularVelocityTarget) {
        const result = simplifiedJoint.getRotatedAxisAndSign(drive.axis);
        if (drive.type === "linear") {
            if (drive.velocityTarget !== undefined) {
                linearVelocityTarget[result.axis] = drive.velocityTarget * result.sign;
            }
        } else if (drive.type === "angular") {
            if (drive.velocityTarget !== undefined) {
                angularVelocityTarget[result.axis] = drive.velocityTarget * result.sign * -1; // PhysX angular velocity is in opposite direction of rotation
            }
        }
    }

    _setDrivePositionTarget(physxJoint, simplifiedJoint) {
        const positionTarget = vec3.fromValues(0, 0, 0);
        const angleTarget = quat.create();
        for (const drive of simplifiedJoint.drives) {
            const result = simplifiedJoint.getRotatedAxisAndSign(drive.axis);
            if (drive.type === "linear") {
                if (drive.positionTarget !== undefined) {
                    positionTarget[result.axis] = drive.positionTarget * result.sign;
                }
            } else if (drive.type === "angular") {
                if (drive.positionTarget !== undefined) {
                    // gl-matrix seems to apply rotations clockwise for positive angles, gltf uses counter-clockwise
                    switch (result.axis) {
                        case 0: {
                            quat.rotateX(
                                angleTarget,
                                angleTarget,
                                -drive.positionTarget * result.sign
                            );
                            break;
                        }
                        case 1: {
                            quat.rotateY(
                                angleTarget,
                                angleTarget,
                                -drive.positionTarget * result.sign
                            );
                            break;
                        }
                        case 2: {
                            quat.rotateZ(
                                angleTarget,
                                angleTarget,
                                -drive.positionTarget * result.sign
                            );
                            break;
                        }
                    }
                }
            }
        }
        const posTarget = new this.PhysX.PxVec3(...positionTarget);
        const rotTarget = new this.PhysX.PxQuat(...angleTarget);
        const targetTransform = new this.PhysX.PxTransform(posTarget, rotTarget);
        physxJoint.setDrivePosition(targetTransform);
        this.PhysX.destroy(posTarget);
        this.PhysX.destroy(rotTarget);
        this.PhysX.destroy(targetTransform);
    }

    createSimplifiedJoint(gltf, node, joint, simplifiedJoint) {
        const resultA = this.computeJointOffsetAndActor(node, simplifiedJoint);
        const resultB = this.computeJointOffsetAndActor(
            gltf.nodes[joint.connectedNode],
            simplifiedJoint
        );

        const pos = new this.PhysX.PxVec3(...resultA.offsetPosition);
        const rot = new this.PhysX.PxQuat(...resultA.offsetRotation);
        const poseA = new this.PhysX.PxTransform(pos, rot);
        this.PhysX.destroy(pos);
        this.PhysX.destroy(rot);

        const posB = new this.PhysX.PxVec3(...resultB.offsetPosition);
        const rotB = new this.PhysX.PxQuat(...resultB.offsetRotation);
        const poseB = new this.PhysX.PxTransform(posB, rotB);
        this.PhysX.destroy(posB);
        this.PhysX.destroy(rotB);

        const physxJoint = this.PhysX.PxTopLevelFunctions.prototype.D6JointCreate(
            this.physics,
            resultA.actor,
            poseA,
            resultB.actor,
            poseB
        );
        this.PhysX.destroy(poseA);
        this.PhysX.destroy(poseB);

        physxJoint.setAngularDriveConfig(this.PhysX.PxD6AngularDriveConfigEnum.eSWING_TWIST);

        physxJoint.setConstraintFlag(
            this.PhysX.PxConstraintFlagEnum.eVISUALIZATION,
            this.debugJoints
        );

        physxJoint.setConstraintFlag(
            this.PhysX.PxConstraintFlagEnum.eCOLLISION_ENABLED,
            joint.enableCollision
        );

        // Do not restict any axis by default
        physxJoint.setMotion(this.PhysX.PxD6AxisEnum.eX, this.PhysX.PxD6MotionEnum.eFREE);
        physxJoint.setMotion(this.PhysX.PxD6AxisEnum.eY, this.PhysX.PxD6MotionEnum.eFREE);
        physxJoint.setMotion(this.PhysX.PxD6AxisEnum.eZ, this.PhysX.PxD6MotionEnum.eFREE);
        physxJoint.setMotion(this.PhysX.PxD6AxisEnum.eTWIST, this.PhysX.PxD6MotionEnum.eFREE);
        physxJoint.setMotion(this.PhysX.PxD6AxisEnum.eSWING1, this.PhysX.PxD6MotionEnum.eFREE);
        physxJoint.setMotion(this.PhysX.PxD6AxisEnum.eSWING2, this.PhysX.PxD6MotionEnum.eFREE);

        for (const limit of simplifiedJoint.limits) {
            this._setLimitValues(physxJoint, simplifiedJoint, limit);
        }

        this._setTwistLimitValues(physxJoint, simplifiedJoint);
        this._setSwingLimitValues(physxJoint, simplifiedJoint);

        const linearVelocityTarget = vec3.fromValues(0, 0, 0);
        const angularVelocityTarget = vec3.fromValues(0, 0, 0);

        for (const drive of simplifiedJoint.drives) {
            this._setDriveValues(physxJoint, simplifiedJoint, drive);
            this._getDriveVelocityTarget(
                simplifiedJoint,
                drive,
                linearVelocityTarget,
                angularVelocityTarget
            );
        }
        this._setDrivePositionTarget(physxJoint, simplifiedJoint);

        const linVel = new this.PhysX.PxVec3(...linearVelocityTarget);
        const angVel = new this.PhysX.PxVec3(...angularVelocityTarget);
        physxJoint.setDriveVelocity(linVel, angVel);
        this.PhysX.destroy(linVel);
        this.PhysX.destroy(angVel);

        return physxJoint;
    }

    changeDebugVisualization() {
        if (!this.scene || !this.debugStateChanged) {
            return;
        }
        this.debugStateChanged = false;
        this.scene.setVisualizationParameter(
            this.PhysX.eSCALE,
            this.debugColliders || this.debugJoints ? 1 : 0
        );
        this.scene.setVisualizationParameter(
            this.PhysX.eWORLD_AXES,
            this.debugColliders || this.debugJoints ? 1 : 0
        );
        this.scene.setVisualizationParameter(
            this.PhysX.eACTOR_AXES,
            this.debugColliders || this.debugJoints ? 1 : 0
        );
        this.scene.setVisualizationParameter(
            this.PhysX.eCOLLISION_SHAPES,
            this.debugColliders ? 1 : 0
        );
        this.scene.setVisualizationParameter(
            this.PhysX.eJOINT_LOCAL_FRAMES,
            this.debugJoints ? 1 : 0
        );
        this.scene.setVisualizationParameter(this.PhysX.eJOINT_LIMITS, this.debugJoints ? 1 : 0);
        for (const joints of this.nodeToSimplifiedJoints.values()) {
            for (const joint of joints) {
                joint.setConstraintFlag(
                    this.PhysX.PxConstraintFlagEnum.eVISUALIZATION,
                    this.debugJoints
                );
            }
        }
        for (const shapePtr of this.shapeToNode.keys()) {
            const shape = this.PhysX.wrapPointer(shapePtr, this.PhysX.PxShape);
            shape.setFlag(this.PhysX.PxShapeFlagEnum.eVISUALIZATION, this.debugColliders);
        }
    }

    initializeSimulation(
        state,
        staticActors,
        kinematicActors,
        dynamicActors,
        jointNodes,
        triggerNodes,
        independentTriggerNodes,
        nodeToMotion,
        _hasRuntimeAnimationTargets,
        _staticMeshColliderCount,
        _dynamicMeshColliderCount
    ) {
        if (!this.PhysX) {
            return;
        }
        this.nodeToMotion = nodeToMotion;
        this.generateSimpleShapes(state.gltf);
        this.computeFilterData(state.gltf);
        for (let i = 0; i < this.filterData.length; i++) {
            const physXFilterData = this.createPhysXCollisionFilter(i);
            this.physXFilterData.push(physXFilterData);
        }

        const materials = state.gltf.extensions?.KHR_physics_rigid_bodies?.physicsMaterials;
        if (materials !== undefined) {
            for (const gltfMaterial of materials) {
                const physxMaterial = this.createPhysXMaterial(gltfMaterial);
                this.physXMaterials.push(physxMaterial);
            }
        }

        const tmpVec = new this.PhysX.PxVec3(0, -9.81, 0);
        const sceneDesc = new this.PhysX.PxSceneDesc(this.tolerances);
        sceneDesc.set_gravity(tmpVec);
        sceneDesc.set_cpuDispatcher(this.PhysX.DefaultCpuDispatcherCreate(0));
        sceneDesc.set_filterShader(this.PhysX.DefaultFilterShader());
        const sceneFlags = new this.PhysX.PxSceneFlags(
            this.PhysX.PxSceneFlagEnum.eENABLE_CCD | this.PhysX.PxSceneFlagEnum.eENABLE_PCM
        );
        sceneDesc.flags = sceneFlags;

        this.scene = this.physics.createScene(sceneDesc);
        let triggerCallback = undefined;

        if (triggerNodes.length > 0) {
            console.log("Enabling trigger report callback");
            triggerCallback = new this.PhysX.PxSimulationEventCallbackImpl();
            triggerCallback.onTrigger = (pairs, count) => {
                for (const compoundTrigger of state.physicsController.compoundTriggerNodes.values()) {
                    compoundTrigger.added.clear();
                    compoundTrigger.removed.clear();
                }
                console.log("Trigger callback called with", count, "pairs");
                for (let i = 0; i < count; i++) {
                    const pair = this.PhysX.NativeArrayHelpers.prototype.getTriggerPairAt(pairs, i);
                    const triggerShape = pair.triggerShape;
                    const otherShape = pair.otherShape;
                    const triggerNodeIndex = this.shapeToNode.get(triggerShape.ptr);
                    const otherNodeIndex = this.shapeToNode.get(otherShape.ptr);
                    if (pair.status === this.PhysX.PxPairFlagEnum.eNOTIFY_TOUCH_FOUND) {
                        state.graphController.rigidBodyTriggerEntered(
                            triggerNodeIndex,
                            otherNodeIndex,
                            nodeToMotion.get(otherNodeIndex)
                        );
                    } else if (pair.status === this.PhysX.PxPairFlagEnum.eNOTIFY_TOUCH_LOST) {
                        state.graphController.rigidBodyTriggerExited(
                            triggerNodeIndex,
                            otherNodeIndex,
                            nodeToMotion.get(otherNodeIndex)
                        );
                    }
                    const compoundTriggers =
                        state.physicsController.triggerToCompound.get(triggerNodeIndex);
                    if (compoundTriggers !== undefined) {
                        for (const compoundTriggerIndex of compoundTriggers) {
                            const compoundTriggerInfo =
                                state.physicsController.compoundTriggerNodes.get(
                                    compoundTriggerIndex
                                );
                            if (pair.status === this.PhysX.PxPairFlagEnum.eNOTIFY_TOUCH_FOUND) {
                                compoundTriggerInfo.added.add(otherNodeIndex);
                            } else if (
                                pair.status === this.PhysX.PxPairFlagEnum.eNOTIFY_TOUCH_LOST
                            ) {
                                compoundTriggerInfo.removed.add(otherNodeIndex);
                            }
                        }
                    }
                }

                for (const [
                    idx,
                    compoundTrigger
                ] of state.physicsController.compoundTriggerNodes.entries()) {
                    for (const addedNodeIndex of compoundTrigger.added) {
                        if (!compoundTrigger.previous.has(addedNodeIndex)) {
                            compoundTrigger.previous.set(addedNodeIndex, 1);
                            state.graphController.rigidBodyTriggerEntered(
                                idx,
                                addedNodeIndex,
                                nodeToMotion.get(addedNodeIndex)
                            );
                        } else {
                            const currentCount = compoundTrigger.previous.get(addedNodeIndex);
                            compoundTrigger.previous.set(addedNodeIndex, currentCount + 1);
                        }
                    }
                    for (const removedNodeIndex of compoundTrigger.removed) {
                        const currentCount = compoundTrigger.previous.get(removedNodeIndex);
                        if (currentCount > 1) {
                            compoundTrigger.previous.set(removedNodeIndex, currentCount - 1);
                        } else {
                            compoundTrigger.previous.delete(removedNodeIndex);
                            state.graphController.rigidBodyTriggerExited(
                                idx,
                                removedNodeIndex,
                                nodeToMotion.get(removedNodeIndex)
                            );
                        }
                    }
                }
            };

            // All callbacks need to be defined
            triggerCallback.onConstraintBreak = (_constraints, _count) => {};
            triggerCallback.onWake = (_actors, _count) => {};
            triggerCallback.onSleep = (_actors, _count) => {};
            triggerCallback.onContact = (_pairHeaders, _pairs, _count) => {};
            sceneDesc.simulationEventCallback = triggerCallback;
        }

        this.scene = this.physics.createScene(sceneDesc);

        console.log("Created scene");
        const shapeFlags = new this.PhysX.PxShapeFlags(
            this.PhysX.PxShapeFlagEnum.eSCENE_QUERY_SHAPE |
                this.PhysX.PxShapeFlagEnum.eSIMULATION_SHAPE
        );

        const triggerFlags = new this.PhysX.PxShapeFlags(this.PhysX.PxShapeFlagEnum.eTRIGGER_SHAPE);

        for (const node of staticActors) {
            this.createActor(state.gltf, node, shapeFlags, triggerFlags, "static");
        }
        for (const node of kinematicActors) {
            this.createActor(state.gltf, node, shapeFlags, triggerFlags, "kinematic");
        }
        for (const node of dynamicActors) {
            this.createActor(state.gltf, node, shapeFlags, triggerFlags, "dynamic", true);
        }
        for (const node of independentTriggerNodes) {
            if (
                this.nodeToActor.has(node.gltfObjectIndex) ||
                this.nodeToMotion.has(node.gltfObjectIndex)
            ) {
                continue;
            }
            this.createActor(state.gltf, node, shapeFlags, triggerFlags, "trigger", true);
        }
        for (const node of jointNodes) {
            this.createJoint(state.gltf, node);
        }

        this.PhysX.destroy(tmpVec);
        this.PhysX.destroy(sceneDesc);
        this.PhysX.destroy(shapeFlags);
        this.PhysX.destroy(triggerFlags);

        this.debugStateChanged = true;
        this.changeDebugVisualization();
    }

    enableDebugColliders(enable) {
        this.debugColliders = enable;
        this.debugStateChanged = true;
    }

    enableDebugJoints(enable) {
        this.debugJoints = enable;
        this.debugStateChanged = true;
    }

    applyTransformRecursively(gltf, node, parentTransform) {
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

    subStepSimulation(state, deltaTime) {
        // eslint-disable-next-line no-unused-vars
        for (const [nodeIndex, { actor, pxShapeMap }] of this.nodeToActor.entries()) {
            const node = state.gltf.nodes[nodeIndex];
            if (node.dirtyTransform) {
                // Node transform is currently animated
                continue;
            }
            const motion = node.extensions?.KHR_physics_rigid_bodies?.motion;
            if (motion && motion.isKinematic) {
                const linearVelocity = motion.computedLinearVelocity ?? motion.linearVelocity;
                const angularVelocity = motion.computedAngularVelocity ?? motion.angularVelocity;
                if (linearVelocity !== undefined || angularVelocity !== undefined) {
                    const worldTransform = node.physicsTransform ?? node.worldTransform;
                    const targetPosition = vec3.create();
                    targetPosition[0] = worldTransform[12];
                    targetPosition[1] = worldTransform[13];
                    targetPosition[2] = worldTransform[14];
                    let nodeRotation = quat.create();
                    if (node.physicsTransform !== undefined) {
                        mat4.getRotation(nodeRotation, worldTransform);
                    } else {
                        nodeRotation = quat.clone(node.worldQuaternion);
                    }
                    if (linearVelocity !== undefined) {
                        const acceleration = vec3.create();
                        vec3.scale(acceleration, linearVelocity, deltaTime);
                        vec3.transformQuat(acceleration, acceleration, nodeRotation);
                        targetPosition[0] += acceleration[0];
                        targetPosition[1] += acceleration[1];
                        targetPosition[2] += acceleration[2];
                    }
                    if (angularVelocity !== undefined) {
                        // Transform angular velocity from local space to world space
                        // by rotating the velocity axes by the current node rotation.
                        const localX = vec3.fromValues(1, 0, 0);
                        const localY = vec3.fromValues(0, 1, 0);
                        const localZ = vec3.fromValues(0, 0, 1);
                        vec3.transformQuat(localX, localX, nodeRotation);
                        vec3.transformQuat(localY, localY, nodeRotation);
                        vec3.transformQuat(localZ, localZ, nodeRotation);

                        const angularAcceleration = quat.create();
                        const qX = quat.create();
                        const qY = quat.create();
                        const qZ = quat.create();
                        quat.setAxisAngle(qX, localX, angularVelocity[0] * deltaTime);
                        quat.setAxisAngle(qY, localY, angularVelocity[1] * deltaTime);
                        quat.setAxisAngle(qZ, localZ, angularVelocity[2] * deltaTime);
                        quat.multiply(angularAcceleration, qX, angularAcceleration);
                        quat.multiply(angularAcceleration, qY, angularAcceleration);
                        quat.multiply(angularAcceleration, qZ, angularAcceleration);

                        quat.multiply(nodeRotation, angularAcceleration, nodeRotation);
                    }
                    const pos = new this.PhysX.PxVec3(...targetPosition);
                    const rot = new this.PhysX.PxQuat(...nodeRotation);
                    const transform = new this.PhysX.PxTransform(pos, rot);

                    actor.setKinematicTarget(transform);
                    this.PhysX.destroy(pos);
                    this.PhysX.destroy(rot);
                    this.PhysX.destroy(transform);

                    const physicsTransform = mat4.create();
                    mat4.fromRotationTranslation(physicsTransform, nodeRotation, targetPosition);

                    const scaledPhysicsTransform = mat4.create();
                    mat4.scale(scaledPhysicsTransform, physicsTransform, node.worldScale);

                    node.physicsTransform = physicsTransform;
                    node.scaledPhysicsTransform = scaledPhysicsTransform;
                }
            } else if (motion && motion.gravityFactor !== 1.0) {
                const force = new this.PhysX.PxVec3(0, -9.81 * motion.gravityFactor, 0);
                actor.addForce(force, this.PhysX.PxForceModeEnum.eACCELERATION);
                this.PhysX.destroy(force);
            }
        }

        this.scene.simulate(deltaTime);
        if (!this.scene.fetchResults(true)) {
            console.warn("PhysX: fetchResults failed");
        }
    }

    simulateStep(state, deltaTime) {
        if (!this.scene) {
            this.reset = false;
            return;
        }
        if (this.reset === true) {
            this._resetSimulation();
            this.reset = false;
            return;
        }

        this.changeDebugVisualization();

        this.subStepSimulation(state, deltaTime);

        // eslint-disable-next-line no-unused-vars
        for (const [nodeIndex, { actor, pxShapeMap }] of this.nodeToActor.entries()) {
            const node = state.gltf.nodes[nodeIndex];
            const motion = node.extensions?.KHR_physics_rigid_bodies?.motion;
            if (motion && !motion.isKinematic && !node.dirtyTransform) {
                const transform = actor.getGlobalPose();
                const position = vec3.fromValues(transform.p.x, transform.p.y, transform.p.z);
                const rotation = quat.fromValues(
                    transform.q.x,
                    transform.q.y,
                    transform.q.z,
                    transform.q.w
                );

                const physicsTransform = mat4.create();
                mat4.fromRotationTranslation(physicsTransform, rotation, position);

                node.physicsTransform = physicsTransform;

                const rotationBetween = quat.create();

                let parentNode = node;
                while (parentNode.parentNode !== undefined) {
                    parentNode = parentNode.parentNode;
                }

                quat.invert(rotationBetween, node.worldQuaternion);
                quat.multiply(rotationBetween, rotation, rotationBetween);

                const rotMat = mat3.create();
                mat3.fromQuat(rotMat, rotationBetween);

                const scaleRot = mat3.create();
                mat3.fromMat4(scaleRot, node.worldTransform);

                mat3.multiply(scaleRot, rotMat, scaleRot);

                const scaledPhysicsTransform = mat4.create();
                scaledPhysicsTransform[0] = scaleRot[0];
                scaledPhysicsTransform[1] = scaleRot[1];
                scaledPhysicsTransform[2] = scaleRot[2];
                scaledPhysicsTransform[4] = scaleRot[3];
                scaledPhysicsTransform[5] = scaleRot[4];
                scaledPhysicsTransform[6] = scaleRot[5];
                scaledPhysicsTransform[8] = scaleRot[6];
                scaledPhysicsTransform[9] = scaleRot[7];
                scaledPhysicsTransform[10] = scaleRot[8];
                scaledPhysicsTransform[12] = position[0];
                scaledPhysicsTransform[13] = position[1];
                scaledPhysicsTransform[14] = position[2];

                node.scaledPhysicsTransform = scaledPhysicsTransform;
                for (const childIndex of node.children) {
                    const childNode = state.gltf.nodes[childIndex];
                    this.applyTransformRecursively(
                        state.gltf,
                        childNode,
                        node.scaledPhysicsTransform
                    );
                }
            }
        }
    }

    resetSimulation() {
        this.reset = true;
        this.simulateStep({}, 0);
    }

    _resetSimulation() {
        const scenePointer = this.scene;
        this.scene = undefined;
        this.filterData = [];
        for (const physXFilterData of this.physXFilterData) {
            this.PhysX.destroy(physXFilterData);
        }
        this.physXFilterData = [];

        for (const material of this.physXMaterials) {
            material.release();
        }
        this.physXMaterials = [];

        for (const shape of this.simpleShapes) {
            shape.destroy?.();
        }
        this.simpleShapes = [];

        for (const convexMesh of this.convexMeshes) {
            convexMesh.release();
        }
        this.convexMeshes = [];

        for (const triangleMesh of this.triangleMeshes) {
            triangleMesh.release();
        }
        this.triangleMeshes = [];

        for (const joints of this.nodeToSimplifiedJoints.values()) {
            for (const joint of joints) {
                joint.release();
            }
        }
        this.nodeToSimplifiedJoints.clear();

        for (const actor of this.nodeToActor.values()) {
            actor.actor.release();
        }

        this.nodeToActor.clear();

        if (scenePointer) {
            scenePointer.release();
        }

        this.shapeToNode.clear();
    }

    getDebugLineData() {
        if (!this.scene || (this.debugColliders === false && this.debugJoints === false)) {
            return [];
        }
        const result = [];
        const rb = this.scene.getRenderBuffer();
        for (let i = 0; i < rb.getNbLines(); i++) {
            const line = this.PhysX.NativeArrayHelpers.prototype.getDebugLineAt(rb.getLines(), i);

            result.push(line.pos0.x);
            result.push(line.pos0.y);
            result.push(line.pos0.z);
            result.push(line.pos1.x);
            result.push(line.pos1.y);
            result.push(line.pos1.z);
        }
        return result;
    }

    applyImpulse(nodeIndex, linearImpulse, angularImpulse) {
        if (!this.scene) {
            return;
        }
        const motionNode = this.nodeToMotion.get(nodeIndex);
        if (!motionNode) {
            return;
        }
        const actorEntry = this.nodeToActor.get(nodeIndex);
        if (!actorEntry) {
            return;
        }
        const actor = actorEntry.actor;

        const linImpulse = new this.PhysX.PxVec3(...linearImpulse);
        const angImpulse = new this.PhysX.PxVec3(...angularImpulse);
        actor.addForce(linImpulse, this.PhysX.PxForceModeEnum.eIMPULSE);
        actor.addTorque(angImpulse, this.PhysX.PxForceModeEnum.eIMPULSE);
        this.PhysX.destroy(linImpulse);
        this.PhysX.destroy(angImpulse);
    }

    applyPointImpulse(nodeIndex, impulse, position) {
        if (!this.scene) {
            return;
        }
        const motionNode = this.nodeToMotion.get(nodeIndex);
        if (!motionNode) {
            return;
        }
        const actorEntry = this.nodeToActor.get(nodeIndex);
        if (!actorEntry) {
            return;
        }
        const actor = actorEntry.actor;

        const pxImpulse = new this.PhysX.PxVec3(...impulse);
        const pxPosition = new this.PhysX.PxVec3(...position);
        this.PhysX.PxRigidBodyExt.prototype.addForceAtPos(
            actor,
            pxImpulse,
            pxPosition,
            this.PhysX.PxForceModeEnum.eIMPULSE
        );
        this.PhysX.destroy(pxImpulse);
        this.PhysX.destroy(pxPosition);
    }

    rayCast(rayStart, rayEnd) {
        const result = {};
        result.hitNodeIndex = -1;
        if (!this.scene) {
            return result;
        }
        const origin = new this.PhysX.PxVec3(...rayStart);
        const directionVec = vec3.create();
        vec3.subtract(directionVec, rayEnd, rayStart);
        vec3.normalize(directionVec, directionVec);
        const direction = new this.PhysX.PxVec3(...directionVec);
        const maxDistance = vec3.distance(rayStart, rayEnd);

        const hitBuffer = new this.PhysX.PxRaycastBuffer10();
        const hitFlags = new this.PhysX.PxHitFlags(this.PhysX.PxHitFlagEnum.eDEFAULT);

        const queryFilterData = new this.PhysX.PxQueryFilterData();
        queryFilterData.set_flags(
            this.PhysX.PxQueryFlagEnum.eSTATIC | this.PhysX.PxQueryFlagEnum.eDYNAMIC
        );

        const hasHit = this.scene.raycast(
            origin,
            direction,
            maxDistance,
            hitBuffer,
            hitFlags,
            queryFilterData
        );

        this.PhysX.destroy(origin);
        this.PhysX.destroy(direction);
        this.PhysX.destroy(hitFlags);
        this.PhysX.destroy(queryFilterData);

        if (hasHit) {
            const hitCount = hitBuffer.getNbAnyHits();
            if (hitCount > 1) {
                console.warn("Raycast hit multiple objects, only the first hit is returned.");
            }
            const hit = hitBuffer.getAnyHit(0);
            const fraction = hit.distance / maxDistance;
            const hitNormal = vec3.fromValues(hit.normal.x, hit.normal.y, hit.normal.z);
            const hitNodeIndex = this.shapeToNode.get(hit.shape.ptr);
            if (hitNodeIndex === undefined) {
                return result;
            }
            return {
                hitNodeIndex: hitNodeIndex,
                hitFraction: fraction,
                hitNormal: hitNormal
            };
        } else {
            return result;
        }
    }
}

export { NvidiaPhysicsInterface };
