using Unity.Mathematics;
using UnityEngine;

namespace HVR.IK.FullTiger
{
    public class HIKEnvironmental : MonoBehaviour
    {
        public Transform groundingRaycastOrigin;

        public void SampleHips(HIKAvatarDefinition definition, float3 headTargetWorldPosition, quaternion headTargetWorldRotation, out float3 environmentalPos, out quaternion environmentalRot)
        {
            // TODO: This is temporary code
            environmentalPos = headTargetWorldPosition + math.down() * definition.refPoseHipToHeadLength;
            environmentalRot = math.mul(Quaternion.LookRotation(math.mul(headTargetWorldRotation, math.down())),
                // This is not a normal reorienter, the LookRotation above does not represent "looking at the next bone"
                // but instead represents where the front of the hips is pointing towards
                MbusGeofunctions.FromToOrientation(math.right(), math.up(), math.forward(), math.left()));
        }
    }
}