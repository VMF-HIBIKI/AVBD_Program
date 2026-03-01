# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Unity-based physics engine implementing **AVBD (Augmented Vertex Block  Descent)**, a position-based dynamics solver with penalty-based constraints. The project focuses on accurate collision detection and stable constraint solving for real-time applications.

### Core Technologies

- Unity 2022.3.25f1 with Burst Compiler for performance-critical code
- Unity Jobs System for parallel processing
- Unity Mathematics (float3, quaternion, float3x3) for all math operations
- NativeArray/NativeList for zero-garbage memory management

## Running and Testing

This is a standard Unity project with no custom build scripts or unit tests.

### To Run

1. Open the project in Unity Editor (2022.3.25f1)
2. Open `Assets/Scenes/SampleScene.unity`
3. Press Play

### To Test Physics

1. Create a GameObject with a `MeshFilter` component
2. Add the `DetectionBody` component (auto-registers with ProcessSystem)
3. Ensure a `ProcessSystem` component exists in the scene
4. Adjust physics parameters in the ProcessSystem inspector:
   - `alpha`: Warm-start factor (0-1)
   - `gamma`: Force decay factor (0-0.99)
   - `friction`: Friction coefficient
   - `beta`: Penalty stiffness reduction
   - `ConstraintSolverIteratorCount`: Solver iterations (1-150)
   - `Gravity`: Gravity magnitude

### Debug Visualization

Toggle debug draws in ProcessSystem inspector:

- `DrawOctree`: Visualize Morton code spatial partitioning
- `DrawConvex`: Show convex hull wireframes for colliding pairs
- `DrawCollision`: Show contact manifolds and normals

## Architecture

### Directory Structure

```
Assets/Scripts/
├── AVBD/                    # Core physics solver
│   ├── AvbdSolver.cs        # Physics parameters container
│   ├── Force.cs             # Contact force with friction, warm-starting
│   └── MatrixUtils.cs       # LDL decomposition, polar decomposition
├── BroadPhase/Jobs/          # Broad phase collision detection
│   ├── BroadPhaseJob.cs     # Morton code-based broad phase
│   ├── MeshToAABBJob.cs     # Mesh to AABB conversion
│   ├── MortonCodeJob.cs     # Generate Morton codes
│   └── SortJob.cs          # Sort by Morton codes
├── Collision/               # Narrow phase collision detection
│   ├── GJK_EPA.cs           # GJK + EPA + Sutherland-Hodgman
│   └── ManifoldPoint.cs     # Contact manifold structure
├── Convex/                  # Convex hull data structures
│   ├── ConvexConstructor.cs # Mesh to convex conversion
│   ├── NativeConvex.cs      # Native convex wrapper
│   └── DataStructures/      # Half-edge structures (Hull, Vertex, Edge, Plane)
├── Tests/                   # Physics bodies and processing
│   ├── DetectionBody.cs     # Rigid body component
│   └── ProcessSystem.cs     # Main physics processing system
└── Utils/                   # Debug visualization tools
```

### Physics Pipeline (ProcessSystem)

The physics simulation runs in `Update()` in this order:

1. **Broad Phase** (`UpdateBroadPhaseProcess`):

   - Convert meshes to world-space AABBs
   - Generate Morton codes for spatial sorting
   - Sort objects by Morton code
   - Find potential collision pairs using spatial hierarchy
2. **Convex Construction** (`CreatConvexesByPairs`):

   - Convert colliding meshes to convex representations
   - Cache convex pairs to avoid recomputation
3. **Narrow Phase** (`NarrowPhaseProcess`):

   - Run GJK to detect collision
   - Run EPA to compute penetration depth and normal
   - Build contact manifold using Sutherland-Hodgman clipping
   - Apply contact forces to both bodies
4. **Physics Update** (`UpdateBodies`):

   - Predict next position/rotation using explicit integration
   - Solve constraints iteratively with warm-starting
   - Update transform to solved position

### Key Algorithms

#### Broad Phase: Morton Code Spatial Sorting

- Converts 3D positions to 1D Morton codes (Z-order curve)
- Sorts objects by Morton code for spatial coherence
- Traverses sorted list to find AABB overlaps
- Configurable `MaxDepth` (2-20) controls octree resolution

#### Narrow Phase: GJK + EPA

- **GJK** (Gilbert-Johnson-Keerthi): Detects collision using Minkowski difference
- **EPA** (Expanding Polytope Algorithm): Computes penetration depth and contact normal
- **Sutherland-Hodgman**: Clips incident face against reference face to generate contact manifold with multiple points

#### Constraint Solver: Position-Based Dynamics

- Penalty-based constraint solving with iterative method
- Warm-starting: Reuses previous frame's constraint forces
- 6x6 LDL decomposition for coupled position-rotation solve
- Coulomb friction model with stick/slip detection

### Important Data Flow

**Force Warm-Starting**:

- Forces are cached by contact hash in `DetectionBody.lastFrameForces`
- Previous frame's lambda (constraint force) is scaled by `alpha * gamma`
- For sticking contacts: full reuse of lambda
- For slipping contacts: decayed reuse

**Memory Management**:

- All NativeArrays/NativeLists use `Allocator.Persistent` for long-lived data
- Temporary allocations use `Allocator.Temp`
- Convexes are disposed each frame after narrow phase
- Always call `Dispose()` on NativeContainers when done

**Transform Updates**:

- DetectionBody maintains separate `position`/`rotation` (current) and `predictedPosition`/`predictedRotation` (next frame)
- Velocity is computed as finite difference: `(predicted - current) / deltaTime`
- Final transform update happens after constraint solving

## Common Tasks

### Adding a New Physics Body

```csharp
// DetectionBody auto-registers with ProcessSystem on Awake
GameObject body = new GameObject("Body");
body.AddComponent<MeshFilter>().sharedMesh = mesh;
body.AddComponent<MeshRenderer>();
var detectionBody = body.AddComponent<DetectionBody>();
detectionBody.mass = 1.0f;
detectionBody.friction = 0.5f;
detectionBody.isStatic = false;
```

### Modifying Physics Parameters

Adjust in ProcessSystem inspector:

- Increase `ConstraintSolverIteratorCount` for more stable stacking (at performance cost)
- Lower `beta` (default 10-100000) for softer contacts
- Adjust `alpha` (0-1) and `gamma` (0-0.99) for warm-starting behavior

### Debugging Collision Issues

1. Enable `DrawOctree` to verify broad phase pairs
2. Enable `DrawConvex` to verify convex hull representation
3. Enable `DrawCollision` to see contact manifolds
4. Check Unity Console for collision pair logs

## Code Conventions

- All math uses `Unity.Mathematics` types: `float3`, `quaternion`, `float3x3`
- Use `math.` prefix for math operations: `math.dot()`, `math.cross()`, `math.normalize()`
- Burst-compiled jobs are marked with `[BurstCompile]`
- Native containers must be disposed manually
- Quaternion multiplication order: `math.mul(current, delta)` applies delta after current
- Commented code is often kept for reference (common in development)

## Implementation Notes

### Sutherland-Hodgman Algorithm (GJK_EPA.cs:462-594)

- Generates contact manifolds by clipping incident face against reference face
- Uses feature face detection (most aligned plane with collision normal)
- Creates side planes from reference face edges for clipping
- Produces multiple contact points for stable stacking

### Warm-Starting (DetectionBody.cs:117-130)

- Critical for simulation stability and convergence
- Hashes contact by otherBodyId + contactPoint positions
- Distinguishes stick vs slip friction for lambda reuse
- Prevents force accumulation errors across frames

### 6x6 Constraint Solve (DetectionBody.cs:185-189)

- Couples position (3 DOF) and rotation (3 DOF)
- Uses manual LDL decomposition (not MatrixUtils.InverseLDL)
- Solves for delta position and delta rotation simultaneously
- Accounts for mass, inertia, and all constraint Jacobians
