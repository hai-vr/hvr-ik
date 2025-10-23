using Unity.Mathematics;
using UnityEngine;

namespace HVR.IK.FullTiger
{
    public class HIKEnvironmental : MonoBehaviour
    {
        public Transform groundingRaycastOrigin;
        public LayerMask groundingRaycastLayerMask;
        
        public bool debugDraw;
        public float useSphereCastRadius;
        
        // This is not a normal reorienter, the LookRotation above does not represent "looking at the next bone"
        // but instead represents where the front of the hips is pointing towards
        private readonly quaternion _lookReorienting = MbusGeofunctions.FromToOrientation(math.right(), math.up(), math.forward(), math.left());

        public void SampleHips(HIKAvatarDefinition definition, float3 headTargetWorldPosition, quaternion headTargetWorldRotation, out float3 environmentalPos, out quaternion environmentalRot)
        {
            // Raycast down from grounding origin
            var raycastHit = false;
            float3 raycastHitPos = float3.zero;
            
            float3 raycastOrigin = (null == groundingRaycastOrigin ? transform : groundingRaycastOrigin).position;
            if (groundingRaycastOrigin != null)
            {
                var raycastDirection = math.down();
                var ray = new Ray(raycastOrigin, raycastDirection);

                if (useSphereCastRadius <= 0f
                        ? Physics.Raycast(ray, out var hit, math.INFINITY, groundingRaycastLayerMask)
                        : Physics.SphereCast(ray, useSphereCastRadius, out hit, math.INFINITY, groundingRaycastLayerMask))
                {
                    raycastHit = true;
                    raycastHitPos = hit.point;
                }
            }
            
            var planarLookDirection = math.mul(headTargetWorldRotation, math.down());
            planarLookDirection.y = 0f;

            var raycastLen = math.length(raycastHitPos - headTargetWorldPosition);
            
            var minimumDistanceToGround = definition.refPoseHipToHeadLength * 0.3f;
            var allowedLengthToGround = definition.refPoseHipToHeadLength + minimumDistanceToGround;
            if (raycastHit && raycastLen < allowedLengthToGround)
            {
                // FIXME: We should do this differently mate, the length of the diagonal is variable
                var towards = math.down() * raycastLen + planarLookDirection * (allowedLengthToGround - raycastLen) + math.up() * minimumDistanceToGround;
                environmentalPos = headTargetWorldPosition + towards;
                
                var acos = math.acos(-math.dot(math.normalize(towards), math.up()));
                environmentalRot = math.mul(math.mul(Quaternion.LookRotation(planarLookDirection), hvr_godot_helper_quaternion.left_hand_AxisAngle(math.right(), -acos * (math.PI / 4f))), _lookReorienting);
            }
            else
            {
                environmentalPos = headTargetWorldPosition + math.down() * definition.refPoseHipToHeadLength;
                environmentalRot = math.mul(Quaternion.LookRotation(planarLookDirection), _lookReorienting);
            }
            
#if UNITY_EDITOR && true
            if (debugDraw)
            {
                MbusUtil.DrawArrow(headTargetWorldPosition, environmentalPos, Color.white, 0f, false, math.mul(environmentalRot, math.forward()));
                if (raycastHit)
                {
                    MbusUtil.DrawArrow(raycastOrigin, raycastHitPos, Color.yellow, 0f, false, math.mul(environmentalRot, math.forward()));
                }
                else
                {
                    MbusUtil.DrawDirectionArrow(raycastOrigin, raycastOrigin + math.down(), Color.yellow, 0f, false, math.mul(environmentalRot, math.forward()));
                }
            }
#endif
        }
    }
}