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
        private readonly quaternion _feetReorienting = MbusGeofunctions.FromToOrientation(math.right(), math.down(), math.forward(), math.right());
        
        private bool _raycastHit;
        private float3 _raycastHitPos;
        private float3 _raycastOrigin;
        private float _raycastLen;
        private float3 _planarLookDirection;

        internal void PerformRaycast(float3 headTargetWorldPosition, quaternion headTargetWorldRotation)
        {
            // Raycast down from grounding origin
            _raycastHit = false;
            _raycastHitPos = float3.zero;
            
            _raycastOrigin = (null == groundingRaycastOrigin ? transform : groundingRaycastOrigin).position;
            if (groundingRaycastOrigin != null)
            {
                var raycastDirection = math.down();
                var ray = new Ray(_raycastOrigin, raycastDirection);

                if (useSphereCastRadius <= 0f
                        ? Physics.Raycast(ray, out var hit, math.INFINITY, groundingRaycastLayerMask)
                        : Physics.SphereCast(ray, useSphereCastRadius, out hit, math.INFINITY, groundingRaycastLayerMask))
                {
                    _raycastHit = true;
                    _raycastHitPos = hit.point;
                }
            }
            
            _planarLookDirection = math.mul(headTargetWorldRotation, math.down());
            _planarLookDirection.y = 0f;

            _raycastLen = math.length(_raycastHitPos - headTargetWorldPosition);
        }

        public void SampleHips(HIKAvatarDefinition definition, float3 headTargetWorldPosition, quaternion headTargetWorldRotation, out float3 environmentalPos, out quaternion environmentalRot)
        {
            var minimumDistanceToGround = definition.refPoseHipToHeadLength * 0.3f;
            var allowedLengthToGround = definition.refPoseHipToHeadLength + minimumDistanceToGround;
            if (_raycastHit && _raycastLen < allowedLengthToGround)
            {
                // FIXME: We should do this differently mate, the length of the diagonal is variable
                var towards = math.down() * _raycastLen + _planarLookDirection * (allowedLengthToGround - _raycastLen) + math.up() * minimumDistanceToGround;
                environmentalPos = headTargetWorldPosition + towards;
                
                var acos = math.acos(-math.dot(math.normalize(towards), math.up()));
                environmentalRot = math.mul(math.mul(Quaternion.LookRotation(_planarLookDirection), hvr_godot_helper_quaternion.left_hand_AxisAngle(math.right(), -acos * (math.PI / 4f))), _lookReorienting);
            }
            else
            {
                environmentalPos = headTargetWorldPosition + math.down() * definition.refPoseHipToHeadLength;
                environmentalRot = math.mul(Quaternion.LookRotation(_planarLookDirection), _lookReorienting);
            }
            
#if UNITY_EDITOR && true
            if (debugDraw)
            {
                MbusUtil.DrawArrow(headTargetWorldPosition, environmentalPos, Color.white, 0f, false, math.mul(environmentalRot, math.forward()));
                if (_raycastHit)
                {
                    MbusUtil.DrawArrow(_raycastOrigin, _raycastHitPos, Color.yellow, 0f, false, math.mul(environmentalRot, math.forward()));
                }
                else
                {
                    MbusUtil.DrawDirectionArrow(_raycastOrigin, _raycastOrigin + math.down(), Color.yellow, 0f, false, math.mul(environmentalRot, math.forward()));
                }
            }
#endif
        }

        public void SampleLeftFoot(HIKAvatarDefinition definition, float3 headTargetWorldPosition, quaternion headTargetWorldRotation, out float3 environmentalPos, out quaternion environmentalRot)
        {
            // TODO: Implement using the raycast result.
            
            var lookRotation = Quaternion.LookRotation(_planarLookDirection);
            environmentalPos = headTargetWorldPosition
                               + math.down() * (definition.refHipsToFootHeight + definition.refPoseHipToHeadLength)
                               + math.mul(lookRotation, math.left()) * 0.05f;
            environmentalRot = math.mul(lookRotation, _feetReorienting);
        }

        public void SampleRightFoot(HIKAvatarDefinition definition, float3 headTargetWorldPosition, quaternion headTargetWorldRotation, out float3 environmentalPos, out quaternion environmentalRot)
        {
            // TODO: Implement using the raycast result.
            
            var lookRotation = Quaternion.LookRotation(_planarLookDirection);
            environmentalPos = headTargetWorldPosition
                               + math.down() * (definition.refHipsToFootHeight + definition.refPoseHipToHeadLength)
                               + math.mul(lookRotation, math.right()) * 0.05f;
            environmentalRot = math.mul(lookRotation, _feetReorienting);
        }
    }
}