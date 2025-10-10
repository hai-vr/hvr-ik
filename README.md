HVR IK
===

An work-in-progress IK solver primarily designed for use in social VR applications, where the inputs of the system are imperfect.

This is the third IK solver that I'm writing, and I hope that it will become good enough to be the last of mine (it probably won't be).

## Adding to the scene

In Edit mode:

- Have an avatar ready in the scene with an *Animator* component in it.
- In a GameObject, create a **HIK Effectors** component.
    - Set the *Animator* field to the avatar animator.
    - None of the other fields matter, leave them empty.
- In a GameObject, create a **HIK Full Tiger** component.
    - Set the *Animator* field to the avatar animator.
    - Set the *Effectors* field to the *HIK Effectors* component created above.

Then start Play mode. The hierarchy under *HIK Effectors* will be filled with objects that represent the end effectors.

## How this solver will work

Writing an IK solver for social VR has two parts:
- Suggest a pose (this is the hard part),
- Solve it (this is the easy part).

### The easy part: Position and rotation solver

This does *not* use a complex IK solver algorithm with multi-effectors and angle limits and such.

We do not use the previous solve as an initial position, but in some cases we may choose to carry some information from the previous solve.
- A) Solve the position of the hips, spine, chest, neck, head, based solely on the hips, and head end effectors (with chest if available).
- B) Solve the roll of the hips, spine, chest, neck, head.
- C) Move the hips-spine-chest-neck-head by the difference between the solved head position and head effector.
- D) Solve the position of the arms and legs. (**Not implemented**)
- E) Solve the roll of the arms and legs. (**Not implemented**)

**A** is done using a combination of Fabrik (*"Andreas Aristidou, Joan Lasenby, FABRIK: A fast, iterative solver for the Inverse Kinematics
problem, Graphical Models, 73 (2011)"*) and custom heuristrics; because the sum of the lengths of the bones on the hips-spine-chest-neck chain is
not necessarily the desired maximum hips-neck distance.

**C** needs to be done so that the HMD lines up with the avatar head. This also gives the initial conditions for solving the arms and legs.

**D** is a triangle, so it's a simple two-bone solver.

In **A** and **D** we do not have to concern ourselves with the rotation of the bones, only the distance between the bones matter.

In **B** and **E** we calculate the rotation of the bones. Two degrees of freedom are constrained by the position solved by **A** and **D**,
so we just have to solve the roll of the bones, also known as twist.

### The hard part: Correction, priming, and bend direction

Solving the position and rotation is the easy part because we don't have to deal with a constraint system with angle limits, and we don't have to
satisfy multiple end effectors. The solver is one solve followed by four other separate solves that don't interfere wtih each other.

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
