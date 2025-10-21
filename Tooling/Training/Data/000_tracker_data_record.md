## Training data

Arm moving around in lookup space.

Right arm moving around, scratching back, neck, head from behind, from front, various movements.

Recorded by Haï~ on 2025-10-20, using two Tundra Trackers and a Valve Index right hand controller.

- Unity coordinate system.
- Data is centered around the primed position of a tracker attached to the shoulder.
- The 6dof tracker for B is attached to the LOWER ARM; this is unlike where you would normally wear it for regular games.
  - This was chosen so that the captured B quaternion would be a better representation of the Lower Arm bone.
- +X is roughly the direction of the right arm when T-posed.
- Approximately scaled where 1 is beyond the maximum length of the arm.
- Quaternions are similar to the post-rotation orientation of the right hand, where +X goes to the tip, +Y is the opposite of the palm,
  and +Z describes the main rotation axis of the parent-current joint.

- A is the primed position (represented at (0,0,0))
- B is the primed position of the arm bend (represented at (0,0,0))
- C is the primed position of the hand (represented at (0,0,0))

### Columns

- b_primed_local.position.X
- b_primed_local.position.Y
- b_primed_local.position.Z
- c_primed_local.position.X
- c_primed_local.position.Y
- c_primed_local.position.Z
- b_primed_local.rotation.quaternion.X
- b_primed_local.rotation.quaternion.Y
- b_primed_local.rotation.quaternion.Z
- b_primed_local.rotation.quaternion.W
- c_primed_local.rotation.quaternion.X
- c_primed_local.rotation.quaternion.Y
- c_primed_local.rotation.quaternion.Z
- c_primed_local.rotation.quaternion.W

### Capture

```csharp
GetPrimedPos(out var aprime, out var bprime, out var cprime);
GetPrimedRot(out var ar, out var br, out var cr);

// Caution: Order matters. This changes the local transform, causing *primelocal to be centered around aprime.
transform.position = aprime;

var bprimelocal = transform.InverseTransformPoint(bprime);
var cprimelocal = transform.InverseTransformPoint(cprime);
var bprimerotationlocal = (Quaternion.Inverse(transform.rotation) * br);
var cprimerotationlocal = (Quaternion.Inverse(transform.rotation) * cr);
string csvdata = $"{bprimelocal.x:0.000000},{bprimelocal.y:0.000000},{bprimelocal.z:0.000000},{cprimelocal.x:0.000000},{cprimelocal.y:0.000000},{cprimelocal.z:0.000000},{bprimerotationlocal.x:0.000000},{bprimerotationlocal.y:0.000000},{bprimerotationlocal.z:0.000000},{bprimerotationlocal.w:0.000000},{cprimerotationlocal.x:0.000000},{cprimerotationlocal.y:0.000000},{cprimerotationlocal.z:0.000000},{cprimerotationlocal.w:0.000000}";

//

private void GetPrimedPos(out Vector3 aprime, out Vector3 bprime, out Vector3 cprime)
{
    aprime = A.position + A.rotation * Vector3.back * 0.05f;
    bprime = B.position + B.rotation * Vector3.back * 0.05f;
    cprime = C.position + C.rotation * Vector3.back * 0.1f;
}

private void GetPrimedRot(out Quaternion ar, out Quaternion br, out Quaternion cr)
{
    ar = A.rotation * FromToOrientation(Vector3.right, Vector3.down, Vector3.forward, Vector3.left);
    br = B.rotation * FromToOrientation(Vector3.right, Vector3.left, Vector3.forward, Vector3.down);
    cr = C.rotation
                 * FromToOrientation(Vector3.right, Vector3.down, Vector3.forward, Vector3.left)
                 * Quaternion.Euler(0, 0, 45)
                 * FromToOrientation(Vector3.up, Vector3.back, Vector3.forward, Vector3.up);
}
```
