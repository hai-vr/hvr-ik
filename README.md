HVR IK
===

A work-in-progress IK solver primarily designed for use in social VR applications, where the inputs of the system are imperfect.

This is the third IK solver that I'm writing, and I hope that it will become good enough to be the last of mine (it probably won't be).

https://github.com/user-attachments/assets/f4b17e25-f894-47cd-a856-1171d6773d23

## Adding to the scene

*Note: This solver has only been tested in Unity 6.2, but it may be modified to ensure compatibility with Unity 2022 in the future.*

In Edit mode:

- Have an avatar ready in the scene with an *Animator* component in it.
- In a GameObject, create a **HIK Effectors** component.
    - Set the *Animator* field to the avatar animator.
    - None of the other fields matter, leave them empty.
- In a GameObject, create a **HIK Full Tiger** component.
    - Set the *Animator* field to the avatar animator.
    - Set the *Effectors* field to the *HIK Effectors* component created above.

Then start Play mode. The hierarchy under *HIK Effectors* will be filled with objects that represent the end effectors.

During Play mode, you can copy the modified GameObject hierarchy containing the *HIK* components and paste it back in Edit mode,
so that you can replay the same pose.

## Options

The *HIK Effectors* component has options that can change the behavior of the solver:
- **Hip Position Matters More** (defaults to false):
  - When true, ensures that the hip position is always reached. The head may not longer be aligned with the HMD when it is too far from it.
  - When false, ensures that the head position is always reached, even if it means moving the hips away from its effector.
    This is designed to make sure that the head is always attached to the HMD.
- **Contortionist** (defaults to false):
  - When true, removes the minimum distance limit between the head and the hips.
  - When false, there is a minimum distance between the head and the hips, which depends on the head angle.
    This is designed to avoid odd looking poses where the spine chain is crunched on itself.
- **Use Straddling Left/Right Leg** (defaults to false):
  - When true, the upper leg will always point towards the lower leg effector, and the lower leg will point towards the foot effector; the foot position will no longer match.
    This is designed to enable poses where grounding the knees matters more than accurately positioning the feet.
  - When false, the lower leg effector only suggests the direction of the bend, and the foot position will match.
- **Use Chest**:
  - When closest to 1, the chest position and rotation will be influenced by the chest effector.
  - When closest to 0, the chest position and rotation will be chosen by default as if there was no chest effector.
- **Also Use Chest to Move Neck**:
  - When this is closest to 1 *and Use Chest* is also closest to 1, the chest position and rotation will influence the position of the neck, which may tilt the upper body.
  - Otherwise, the neck position will be solved by default as if there was no chest effector.
- **Use L/R Lower Arm**:
  - When closest to 1, the arm will bend towards the lower arm effector.
  - When closest to 0, the arm will bend using the default arm bend direction heuristics.

### Parenting the hand effectors to the avatar using self-parenting

The left hand effector and the right hand effector can be virtually parented to the avatar's humanoid bones, using the **self-parenting** function.
This differs from moving the effectors to the avatar's bones prior to solving.

The main use case for this is to enable the ability to grab your own hip or grab your own leg. Grabbing your own arm is much more complicated;
you are strongly discouraged from enabling the ability to do this using this version of the solver.

By modifying the relative position and rotation dynamically prior to solving, it may be possible to virtually parent to child transforms located within the hierarchy
of the humanoid bone in question.

- *Use Self Parent Hand*: When closest to 1, the hand effector will be virtually moved to the bone specified in *Self Parent Hand Bone*.
- *Self Parent Hand Bone*: Defines the bones to parent to.
- *Self Parent Relative Position*: The relative position between that bone and the effector.
- *Self Parent Relative Rotation*: The relative rotation between that bone and the effector.

## Solving without Transforms

The IK solver was designed to run without any access to Transform, GameObject, nor Component hierarchy during the solve.

The *HIK Effectors* and *HIK Full Tiger* components are just convenient accessors for the actual solver functionnality, but they don't contain
critical logic in them.

If you instantiate the following classes directly, you can request a solve without needing Components nor Transforms:
- **HIKAvatarDefinition** contains a dump of all the avatar-specific data.
  - You can learn how to fill this by looking at the implementation of *HIK Full Tiger*'s `SolveDefinition` static function.
- **HIKSnapshot** will contain the result of the solve.
  - You can learn how to read data out of it by looking at the implementation of *HIK Full Tiger*'s `ApplySnapshot` function.
- **HIKObjective** contains the parameters of the solver (effectors, options, etc.).
  - You can learn how to fill this by looking at the implementation of *HIK Effectors* `CreateTarget` function, and *HIK Full Tiger*'s `PerformRegularSolve` function.
- **HIKSolver** is the solver you need to instantiate using all the aforementioned classes.

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

In **A** and **D** we do not have to concern ourselves with the rotation of the bones, only the distance between the bones matter.

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

There is a case where we will give the solver a position that is solvable, but would alter the artistic vision of the avatar.
- Some avatars have a spine chain that is naturally curved by default in its rest pose. This causes the distance between the hip and the neck bone
  to be shorter than the sum of the bones between the hips-spine-chest-neck chain.
- In that case, we should avoid straightening the spine chain, because it causes the avatar mesh to buckle inwards or outwards into an unnatural appearance.

#### Priming the solver

IK solvers for non-triangles are not great at converging to a solution if the initial position is made of colinear points.

We need to prime the initial position of the solver with an estimation of how the spine will be shaped like, before we give it to the solver.

The solver is here to ensure that the lengths between the bones are correct, but priming provides the shape of the spine.

#### Choosing a bend direction

This section is not described yet.
