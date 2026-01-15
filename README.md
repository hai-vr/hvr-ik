HVR IK
===

A work-in-progress IK solver primarily designed for use in social VR applications, where the inputs of the system are imperfect.

This is the third IK solver that I'm writing, and I hope that it will become good enough to be the last of mine (it probably won't be).

*Made for use in Unity 6.2, and partially ported to [Godot Engine .NET](https://github.com/hai-vr/hvr-ik/pull/6). The core part of
this solver [does not require Unity Components nor Transforms](#solving-without-transforms), hence why it works in Godot Engine.*

https://github.com/user-attachments/assets/f4b17e25-f894-47cd-a856-1171d6773d23

### Why?

An IK solver normally refers to a system that rotates bones in a chain to reach a target position. However, in social VR applications, *"IK"*
colloquially refers to both that system but also all the heuristics and tuning necessary that makes an avatar model feel good when worn in first person.

The process of writing an IK solver is already massively documented through mathematical papers and implementations of those papers, but the process
of tuning an IK solver for social VR is much less documented, on top of being completely subjective and different based on the needs of the users of
the application.

There are a lot of ready-to-use IK solvers out there [including in Unity itself](https://docs.unity3d.com/Packages/com.unity.animation.rigging@1.1/manual/ConstraintComponents.html),
but most will not help you with this tuning and are only interested in providing a solution to the literal definition of an IK solver.

The mission of this repository is to document and demystify the decisions related to this tuning and also provides an implementation that you may use to
try and develop your own heuristics for the needs of your own application.

In addition, the core part of this solver [does not require Unity Components nor Transforms](#solving-without-transforms), so with some effort, it may
be ported to other game engines;<br/>As a proof of concept, there is [a branch capable of running on Godot Engine .NET](https://github.com/hai-vr/hvr-ik/pull/6),
which is achieved by aliasing the Unity.Mathematics library with Godot Engine's own types.

> [!WARNING]  
> This solver is currently incomplete as it is a work in progress. The following parts are functional but cannot be considered ready for production:
> - General tuning in first person.
> - Arm bend is rudimentary and its heuristics are currently being re-engineered with a several different approaches.
> - Leg and upper arm twist has pathological issues at acute and extreme angles.
> - A hip target is currently mandatory, but it will not be required in the future.
> - Does not yet support armatures that have an upper chest bone.

## Adding to the scene

> [!NOTE]  
> This solver has only been tested in Unity 6.2, but it may be modified to ensure compatibility with Unity 2022 in the future.
> 
> At this time of writing, this project is primarily intended for use by **software developers**, not end users. If you are a user, you
> can still try the steps below, but it may be a poor experience. Please check back at another time.

In Edit mode:

- Have an avatar ready in the scene with an *Animator* component in it.
- In a GameObject, create a **HIK Effectors** component.
    - Set the *Animator* field to the avatar animator.
    - None of the other fields matter; leave them empty.
- In a GameObject, create a **HIK Full Tiger** component.
    - Set the *Animator* field to the avatar animator.
    - Set the *Effectors* field to the *HIK Effectors* component created above.
    - The *Environmental* field is optional and will be used in the future to extract data from the surrounding environment and provide poses (ground location, etc.)

Then start Play mode. The hierarchy under *HIK Effectors* will be filled with objects that represent the end effectors.

During Play mode, you can copy the modified GameObject hierarchy containing the *HIK* components and paste it back in Edit mode,
so that you can replay the same pose.

As an extra, you can convert a pose to an animation clip. First, open the animation clip editor in a visible tab. Then, in the inspector,
select the *HIK Full Tiger* component and click the *Create animation clip from pose* button. You can then copy the keyframes.

The animation clip may not be a faithful 1:1 copy of the pose as it may handle twist differently, but I am not entirely sure where the discrepancy comes from.

### Animation Rigging

The project currently contains a RigConstraint behaviour called **HVR Full Tiger Animation Rigging**, meant to be used with [Unity's Animation Rigging package](https://docs.unity3d.com/Packages/com.unity.animation.rigging@1.3/manual/index.html).

This class is **not ready** for use, as in it won't work at all; this part is under active development. Please check back another time.

## Options

The *HIK Effectors* component has options that can change the behavior of the solver.

#### Spine-related

- **Hip Position Matters More** (defaults to 0):
  - When closest to 1, ensures that the hip position is always reached. The head may no longer be aligned with the HMD when it is too far from it.
  - When closest to 0, ensures that the head position is always reached, even if it means moving the hips away from its effector.
    This is designed to make sure that the head is always attached to the HMD.
- **Contortionist** (defaults to false):
  - When true, removes the minimum distance limit between the head and the hips.
  - When false, there is a minimum distance between the head and the hips, which depends on the head angle.
    This is designed to avoid odd-looking poses where the spine chain is crunched on itself.
- **Do Not Preserve Hips To Neck Curvature Limit** (defaults to false): See [Not Recommended](#not-recommended) below.
- **Improve Spine Buckling**: Helps overcoming the spine curvature when the body is upright while the hips-to-neck distance is shorter than default. See [the section about spine buckling](#improving-spine-buckling) below.

#### Automatic Chest

- **Chest Rotation Uses Head** (defaults to 0):
  - When closest to 1, the chest rotation uses the Head roll. This has no effect when *Use Chest* is closest to 1.
  - When closest to 0, the chest rotation uses the Hip roll. This has no effect when *Use Chest* is closest to 1.

#### Chest effector

- **Use Chest**:
    - When closest to 1, the chest position and rotation will be influenced by the chest effector.
    - When closest to 0, the chest position and rotation will be chosen by default as if there was no chest effector.
- **Also Use Chest to Move Neck**:
    - When this is closest to 1 *and Use Chest* is also closest to 1, the chest position and rotation will influence the position of the neck, which may tilt the upper body.
    - Otherwise, the neck position will be solved by default as if there was no chest effector.

#### Arm bend

- **Use L/R Lower Arm**:
    - When closest to 1, the arm will bend towards the lower arm effector.
    - When closest to 0, the arm will bend using the default arm bend direction heuristics.

#### Leg bend

- **Use L/R Lower Leg**:
  - When closest to 1, the leg will bend towards the lower leg effector.
  - When closest to 0, the leg will bend using the default leg bend direction heuristics.
  - *This function has no effect if the [Straddling](#straddling) feature is enabled.*

#### Struggle

- **Struggle**: Improves how fast the arm angle opens when the maximum arm length is being reached. See [Struggle section](#struggle) below.

#### Shoulder

- **Use Shoulder** (defaults to 1):
    - When closest to 1, the shoulder rotation may move based on the direction and how far the hand is away from the upper arm root bone position at rest.
    - When closest to 0, the shoulder willl be at its rest position.
- **Shoulder Forward Angle Multiplier**: When the hand is trying to reach in the forward direction of the chest, multiply the default maximum shoulder angle by this value.
- **Shoulder Upward Angle Multiplier**: When the hand is trying to reach in the upward direction of the chest, multiply the default maximum shoulder angle by this value.

#### Straddling
    
- **Use Straddling Left/Right Leg** (defaults to false):
  - When true, the upper leg will always point towards the lower leg effector, and the lower leg will point towards the foot effector; the foot position will no longer match.
    This is designed to enable poses where grounding the knees matters more than accurately positioning the feet.
  - When false, the lower leg effector only suggests the direction of the bend, and the foot position will match.

#### Self-parenting

- **Self-parenting**: Allows parenting the hands to the body or legs. See [the section about self-parenting](#parenting-the-hand-effectors-to-the-avatar-using-self-parenting) below.

#### Environmental

- **Use Hips from Environmental**:
  - When closest to 1, the hips position and rotation will be sampled from the *HIKEnvironmental* component, which gets data from environmental factors such as gravity and colliders.
  - When closest to 0, the hips position and rotation will use the Hips effector.

#### Experimental

- (EXPERIMENTAL) **Use Fake Double Jointed Knees**: This attempts to prevent the upper leg from clipping into the lower leg by moving the position of the lower leg. See [the fake knee joints](#fake-knee-joints) below.

#### Direct drive

- **Use Direct Drive**: For scripting only. When true, none of the effector transforms will be used, and you will need to supply the world position and rotation to hidden fields
  located within the *HIKEffectors* component, only visible through scripting. Enabling this option before the *HIKEffectors* component is enabled will not generate any of the effector transforms.

#### Not recommended

*Not recommended:*
- **Do Not Preserve Hips To Neck Curvature Limit** (defaults to false):
  - Many avatars are designed with a spine that is naturally curved. By default (when this option is false), the solver will do its best to preserve that curve.
    That means that, if the head effector target is further away than the hips, we try to avoid making all the bones of the spine (hips-neck chain) point to it as a straight line.
    This option overrides this behavior so that the spine can become straight, but this is likely to negatively affect the appearance of the avatar mesh. 
  - When true, we do not preserve the spine curvature by not imposing any maximum distance between the hips and the neck; the maximum distance is effectively the sum of the length of the bones.
  - When false, we preserve the spine curvature by limiting the maximum distance between the hips and the head to be smaller than the hips-to-neck length + the neck-to-head length (which is not equal to the sum of the length of the bones).

### Improving spine buckling

When the hips-to-neck distance is smaller than the default hips-to-neck distance at rest, the spine is considered crunched.
This poses a problem when the body is upright, because the spine needs to curve in a different way when the head is (horizontally) in front of the hips,
versus when the head is behind the hips.

When **Improve Spine Buckling** is closer to 1, the solver will slightly push the hips position down around the location where the head position and direction followed
the default spine curvature. This will straighten the spine so that it has a better opportunity to transform from one curve into the other.

https://github.com/user-attachments/assets/6fca310b-6d36-4aac-99b2-ca237bacf497

### Parenting the hand effectors to the avatar using self-parenting

The left and right hand effectors can be virtually parented to the avatar's humanoid bones, using the **self-parenting** function.
This differs from moving the effectors to the avatar's bones prior to solving.

The main use case for this is to enable the ability to grab your own hip or grab your own leg. Grabbing your own arm is much more complicated;
you are strongly discouraged from enabling the ability to do this using this version of the solver.

By modifying the relative position and rotation dynamically prior to solving, it may be possible to virtually parent to child transforms located within the hierarchy
of the humanoid bone in question.

- *Use Self Parent Hand*: When closest to 1, the hand effector will be virtually moved to the bone specified in *Self Parent Hand Bone*.
- *Self Parent Hand Bone*: Defines the bones to parent to.
- *Self Parent Relative Position*: The relative position between that bone and the effector.
- *Self Parent Relative Rotation*: The relative rotation between that bone and the effector.

### Struggle

The struggle parameter is an attempt to help alleviate the following issue with two-bone IK solvers:

When the end effector is very close to the maximum distance, the last few percent lead to an acceleration in the change of the angle of the joint.

The *struggle* parameters try to alleviate this by changing the behaviour so that when the end effector is located between 99% and 104% of the total arm or leg length,
it gets remapped from 99% to 100% following the curve of a charging capacitor.

https://github.com/user-attachments/assets/90fee552-3fbe-4760-9470-f48f4003db29

The default values are 0.99 and 1.04, corresponding to 99% and 104%. To disable this behaviour, use a value of 1 and 1.

<img width="1607" height="1326" alt="armcurve-global" src="https://github.com/user-attachments/assets/9a0b6b66-8c7e-42d9-a930-ff6916fb72ad" />

*When struggle is enabled, the last one percent of the distance takes longer to reach, but it avoids a sudden acceleration of the joint angle in that last percent.*

### Fake knee joints

On traditional avatar rigs, the upper leg and lower leg are two bones connected by a joint. This poses a problem when the angle between the upper leg and
the lower leg is acute, because it causes the thighs to clip into the calves.

Some more complex rigs avoid this by connecting the upper leg and lower leg using multiple joints and may also use multiple accessory bones, but most avatars
designed for social VR are not designed this way for various justifiable reasons (compatibility, constraint complexity, and unsupported number of bones per vertex
on certain apps. Unity *does* support more than 4 bones per vertex on the GPU at a possible performance cost if you create your own application).

The fake knee joints option violates the convention that bones only move by rotating around their axis. This option causes the lower leg to move away from the default
upper-leg-to-lower-leg rotation joint.

https://github.com/user-attachments/assets/de4d9d4f-dca4-4df9-847c-6ca910a25985

> [!CAUTION]
> This option changes the position of the lower leg bone, but also causes a different rotation of the lower leg bone compared to when this option is disabled.
> 
> This can make it difficult to network the pose of the avatar through the IK networking solution of your application, but also animation systems may not be able to handle it.

## Controlling execution order

This component has an option called **updateEveryFrame**, which defaults to true.

When you set this to false, you can manually control the execution order by invoking the `(HIKFullTiger).PerformRegularSolve()` and the `(HIKFullTiger).ApplySnapshot()` functions from your own code.

The solver can also be used without even using *HIK Full Tiger* nor *HIK Effectors*, please [read the section below about solving without transforms](#solving-without-transforms).

## Solving without Transforms

> [!WARNING]  
> This project is under active development and has no stable release at this time of writing. Any class will be subject to breaking changes without any notice,
> no matter whether something is marked as public, internal, or private.
> 
> Current work towards using the Unity Job system and towards compatibility with the Animation Rigging package will particularly influence the instability of the API.

The IK solver was designed to run without any access to Transform, GameObject, nor Component hierarchy during the solve.

The *HIK Effectors* and *HIK Full Tiger* components are just convenient accessors for the actual solver functionality, but they don't contain
critical logic in them.

If you instantiate the following classes directly, you can request a solve without needing Components nor Transforms:
- **HIKAvatarDefinition** contains a dump of all the avatar-specific data.
  - You can learn how to fill this by looking at the implementation of *HIK Full Tiger*'s `SolveDefinition` static function.
- **HIKSnapshot** will contain the result of the solve.
  - You can learn how to read data out of it by looking at the implementation of *HIK Full Tiger*'s `ApplySnapshot` function.
- **HIKObjective** contains the parameters of the solver (effectors, options, etc.).
  - You can learn how to fill this by looking at the implementation of *HIK Effectors* `CreateTarget` function, and *HIK Full Tiger*'s `PerformRegularSolve` function.
- **HIKSolver** is the solver you need to instantiate using all the aforementioned classes.

As a proof of concept, there is [a branch capable of running on Godot Engine .NET](https://github.com/hai-vr/hvr-ik/pull/6).

## Solver considerations

This solver works under the following assumptions:
- Import:
  - This solver can only operate on avatar models that were imported in Unity as Humanoid.
    - With some difficulty, you may be able to bypass this restriction by filling the *HIK Avatar Definition*
      object yourself and [solving without transforms](#solving-without-transforms).
  - There is no need to perform any bone angle normalization (no VRM-like normalization is needed).
- Avatar scale:
  - The Transform hierarchy must be uniformly scaled.
  - You may change the avatar scale at runtime (Hips or any parent of Hips). Effectors operate in world space.
  - You may not change the scale of any child transform of the Hips at runtime.
- Bone positions: 
  - The bone positions in local space must not change at runtime, except for the Hips.

This solver provides the following guarantees:
- The results can be applied by setting the Hips transform position and rotation, and the absolute rotation of all the other bones.
- Unless specified by enabling a special option (not available as of this version of the solver), this solver does not require setting
  the position of any bone other than the hips.
  - You should be able to network the pose of the avatar by transmitting the position and rotation of the Hips transform, and the
    rotation of all other bones.
  - This guarantee is broken if you enable the *Double Joined Knees* in the Experimental section, as this option requires setting the
    position of the lower legs.
- The result of the solver does not depend on the previous pose of the avatar, unless specified (partial solves).
  - You are able to run a solve or several different solves multiple times per frame onto the same avatar if you deem it necessary
    (e.g. a solve to be submitted for networking purposes, followed by a partial local solve for real-time object attachment purposes).
  - The time, delta time, or frame is not an input of the solver. There is no smoothing-over-time function.
- You are not required to apply the result of the solver to any Transform. Solving is an operation that is separate from applying the result.

Also, the current version of the solver does not support avatars that have an UpperChest bone. Please check back at another time.

## How this solver will work

Writing an IK solver for social VR has two parts:
- Suggest a pose (this is the hard part),
- Solve it (this is the easy part).

### The easy part: Position and rotation solver

This does *not* use a complex IK solver algorithm with multi-effectors and angle limits and such.

We do not use the previous solve as an initial position, but in some cases we may choose to carry some information from the previous solve
(for example, the hand and arm angles so that we may guess much they could still twist).
- A) Solve the position of the hips, spine, chest, neck, head, based solely on the hips, and head end effectors (with chest if available).
- B) Solve the roll of the hips, spine, chest, neck, head.
- C) Move the hips-spine-chest-neck-head by the difference between the solved head position and head effector.
- D) Solve the position of the arms and legs.
- E) Solve the roll of the arms and legs.

**A** is done using a combination of Fabrik (*"Andreas Aristidou, Joan Lasenby, FABRIK: A fast, iterative solver for the Inverse Kinematics
problem, Graphical Models, 73 (2011)"*) and custom heuristics; because the sum of the lengths of the bones on the hips-spine-chest-neck chain is
not necessarily the desired maximum hips-neck distance.

**C** needs to be done so that the HMD lines up with the avatar head. This also gives the initial conditions for solving the arms and legs.

**D** is a triangle, so it's a simple two-bone solver.

In **A** and **D** we do not have to concern ourselves with the rotation of the bones, only the distance between the bones matters.

In **B** and **E** we calculate the rotation of the bones. Two degrees of freedom are constrained by the position solved by **A** and **D**,
so we just have to solve the roll of the bones, also known as twist.

### The hard part: Correction, priming, and bend direction

Solving the position and rotation is the easy part because we don't have to deal with a constraint system with angle limits, and we don't have to
satisfy multiple end effectors. The solver is one solve followed by four other separate solves that don't interfere with each other.

The hard part is **making the IK feel good when worn in first person**, and this is completely subjective.

I categorize this into three classes of issues:
- Correcting the effectors,
- Priming the solver,
- Choosing a bend direction.

#### Correcting the effectors

There are cases where we will give the solver a position that does not make sense to solve.
- If the avatar's body is straight in any orientation, but the head is way too close to the hips, it's probably a mistake, so we need to shift the hips down.
- However, if the avatar's body is bent over, then the head can be close to the hips, so it is not considered to be a mistake.

There is a case where we will give the solver a position that is solvable but would alter the artistic vision of the avatar.
- Some avatars have a spine chain naturally curved by default in its rest pose. This causes the distance between the hip and the neck bone
  to be shorter than the sum of the bones between the hips-spine-chest-neck chain.
- In that case, we should avoid straightening the spine chain, because it causes the avatar mesh to buckle inwards or outwards into an unnatural appearance.

#### Priming the solver

IK solvers for more than 4 points are not great at converging to a solution if the initial state of the solver has positions that are colinear to the target effector.

We need to prime the initial position of the solver with an estimation of how the spine will be shaped like, before we give it to the solver.

The solver is here to ensure that the lengths between the bones are correct, but priming provides the shape of the spine.

#### Choosing a bend direction

This section is not described yet.

The bend direction of the arms is a particularly annoying part, and I still have a lot to figure out with this.

In a future version I would like to try using [Motion Matching](https://www.youtube.com/watch?v=KSTn3ePDt50) ([2](https://www.youtube.com/watch?v=RCu-NzH4zrs)) with some arm motion capture
to see if the bend direction could be solved using this technique.
