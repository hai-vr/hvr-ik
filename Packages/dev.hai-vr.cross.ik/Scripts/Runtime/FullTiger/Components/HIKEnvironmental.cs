using Unity.Mathematics;
using UnityEngine;

namespace HVR.IK.FullTiger
{
    public class HIKEnvironmental : MonoBehaviour
    {
        public Transform groundingRaycastOrigin;
        public bool debugDraw;

        public void SampleHips(HIKAvatarDefinition definition, float3 headTargetWorldPosition, quaternion headTargetWorldRotation, out float3 environmentalPos, out quaternion environmentalRot)
        {
            // TODO: This is temporary code
            environmentalPos = headTargetWorldPosition + math.down() * definition.refPoseHipToHeadLength;
            var forward = math.mul(headTargetWorldRotation, math.down());
            forward.y = 0f;
            environmentalRot = math.mul(Quaternion.LookRotation(forward),
                // This is not a normal reorienter, the LookRotation above does not represent "looking at the next bone"
                // but instead represents where the front of the hips is pointing towards
                MbusGeofunctions.FromToOrientation(math.right(), math.up(), math.forward(), math.left()));
#if UNITY_EDITOR && true
            if (debugDraw)
            {
                MbusUtil.DrawArrow(headTargetWorldPosition, environmentalPos, Color.white, 0f, false, math.mul(environmentalRot, math.forward()));
            }
#endif
        }
    }
}