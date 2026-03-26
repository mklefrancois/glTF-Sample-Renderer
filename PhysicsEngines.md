# Supported Physics Engines and Limitations

Currently, only NVIDIA PhysX is supported.

## NVIDIA PhysX

The PhysX engine is loaded as a WebAssembly module via [physx-js-webidl](https://github.com/fabmax/physx-js-webidl).

### Limitations

#### Mesh Colliders

PhysX does not support triangle meshes as collision shapes for dynamic (non-kinematic) actors and triggers. When building a dynamic actor or trigger that references a mesh collider without the `convexHull` flag, the mesh is cooked as a convex hull anyway.

#### Shape Approximations (`KHR_implicit_shapes`)

PhysX does not have native types for every shape defined in the glTF spec. The following shapes are approximated:

- **Cylinder** — PhysX has no native cylinder type. Cylinders are always represented as convex meshes built from generated vertex data.
- **Capsule** — PhysX's native capsule type is limited to equal top/bottom radii and requires X-axis alignment. All glTF capsules are represented as convex meshes to support arbitrary radii and scaling.
- **Sphere with non-uniform scale** — Falls back to a convex mesh approximation. A uniform-scale sphere uses the native `PxSphereGeometry`.
- **Box with non-uniform scale and a non-identity scale-axis quaternion** — Falls back to a convex mesh approximation. Uniformly or simply scaled boxes use the native `PxBoxGeometry`.

#### Joint Simplification

`KHR_physics_rigid_bodies` allows arbitrary per-axis combinations of linear and angular limits and drives. PhysX only exposes D6 joints with a fixed twist/swing constraint model. The implementation resolves this mismatch by:

1. Decomposing a glTF joint into one or more `simplifiedPhysicsJoint` objects, splitting whenever multiple limits affect the same axis.
2. Remapping glTF axis indices to PhysX's expected twist axis (X) via a computed local rotation quaternion.
3. Mapping cone/ellipse limits (`angularAxes` spanning multiple axes) to PhysX swing limits.

This simplification is a best-effort approximation. Complex joint definitions might result in unexpected behavior such as shaking, jittering and instability.

#### Collision Filters

The filter system uses a 32-bit bitmask internally. A maximum of **31 user-defined** collision filters are supported per scene. If a glTF asset defines more than 31 collision filters, filters beyond the limit are ignored and a warning is emitted.
