// Copyright 2025 Haï~ (@vr_hai github.com/hai-vr)
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//    http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
using Unity.Mathematics;
using UnityEngine;
#else //__iff HVR_IS_GODOT
using float3 = Godot.Vector3;
using float2 = Godot.Vector2;
using float4x4 = Godot.Transform3D;
using quaternion = Godot.Quaternion;
using math = hvr_godot_math;
#endif
using static HVR.IK.FullTiger.HIKBodyBones;

namespace HVR.IK.FullTiger
{
    internal class/*was_struct*/ HIKSpineSolver
    {
        public const int Iterations = 20;
        
        private readonly HIKAvatarDefinition definition;
        private readonly quaternion _reorienter;

        private HIKSpineData<float3> _spineChain;
        private HIKSpineData<float> _spineDistances;
        private readonly float _hipsToSpineToCheckToNeckToHeadLength;

#if UNITY_EDITOR && true
        private static readonly Color lawnGreen = new Color(0.4862745f, 0.9882354f, 0.0f, 1f);
        private static readonly Color lawnGreenTransparent = new Color(0.4862745f, 0.9882354f, 0.0f, 0.3f);
        private static readonly Color coral = new Color(1f, 0.4980392f, 0.3137255f, 1f);
        private static readonly Color mediumOrchid = new Color(0.7294118f, 0.3333333f, 0.8274511f, 1f);
#endif

        public HIKSpineSolver(HIKAvatarDefinition definition, quaternion reorienter)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");
            
            this.definition = definition;
            _reorienter = reorienter;

            _spineChain = new HIKSpineData<float3>(4);
            _spineDistances = new HIKSpineData<float>(3);
            _spineDistances[0] = math.distance(definition.refPoseHiplativePos[(int)Spine], definition.refPoseHiplativePos[(int)Chest]);
            _spineDistances[1] = math.distance(definition.refPoseHiplativePos[(int)Chest], definition.refPoseHiplativePos[(int)Neck]);
            _spineDistances[2] = math.distance(definition.refPoseHiplativePos[(int)Neck], definition.refPoseHiplativePos[(int)Head]);

            _hipsToSpineToCheckToNeckToHeadLength = definition.refPoseHipsLength + definition.refPoseSpineLength + definition.refPoseChestLength + definition.refPoseNeckLength;
        }

        public HIKSnapshot Solve(HIKObjective objective, HIKSnapshot ikSnapshot, bool debugDrawSolver)
        {
            var scale = math.length(objective.providedLossyScale) / math.length(definition.capturedWithLossyScale);
            
            var originalHipTargetPos = objective.hipTargetWorldPosition;
            var originalHeadTargetPos = objective.headTargetWorldPosition;
            
            var hipTargetPos = originalHipTargetPos;
            var headTargetPos = originalHeadTargetPos;
            
            // ## Corrections

            if (!objective.allowContortionist)
            {
                var headAndHipSameDirection01 = math.dot(math.mul(objective.headTargetWorldRotation, math.right()), math.mul(objective.hipTargetWorldRotation, math.right())) * 0.5f + 0.5f;
                var ff = math.lerp(0.01f, (definition.refPoseHipToNeckLength + definition.refPoseNeckLength * 0.75f) / definition.refPoseHipToHeadLength, headAndHipSameDirection01) * scale;
                if (math.distance(originalHipTargetPos, headTargetPos) < definition.refPoseHipToHeadLength * ff) // TODO: Allow this to be closer if the head and hip are not in the same direction
                {
                    hipTargetPos = headTargetPos - math.normalize(headTargetPos - originalHipTargetPos) * definition.refPoseHipToHeadLength * ff;
#if UNITY_EDITOR && true
                    if (debugDrawSolver) Debug.DrawLine(headTargetPos, hipTargetPos, Color.red, 0f, false);
#endif
                }
                else
                {
                    var kk = math.normalize(headTargetPos - hipTargetPos) * definition.refPoseHipToHeadLength * ff;
#if UNITY_EDITOR && true
                    if (debugDrawSolver) Debug.DrawLine(hipTargetPos, hipTargetPos + kk, Color.yellow, 0f, false);
#endif
                }
            }

            void ApplyLimiter(ref float3 in_hipTargetPos, ref float3 in_headTargetPos) {
                var maximumLength = 
                    !objective.doNotPreserveHipsToNeckCurvatureLimit
                    // If the distance between the head and the neck is larger than the length of the neck + refPoseHipToNeckLength
                    // (which is not equal to the sum of the bones of the hip-spine-chest-neck) chain, then either the head or the hips MUST be brought closer
                    // so that the solver doesn't overstretch the artists' spine.
                    ? (definition.refPoseHipToNeckLength + definition.refPoseNeckLength) * scale
                    : _hipsToSpineToCheckToNeckToHeadLength * scale;
                if (math.distance(in_hipTargetPos, in_headTargetPos) > maximumLength)
                {
                    if (objective.headAlignmentMattersMore)
                    {
                        var toHip = math.normalize(in_hipTargetPos - in_headTargetPos);
                        in_hipTargetPos = in_headTargetPos + toHip * maximumLength;
                    }
                    else
                    {
                        var toHead = math.normalize(in_headTargetPos - in_hipTargetPos);
                        in_headTargetPos = in_hipTargetPos + toHead * maximumLength;
                    }
                }
            }
            ApplyLimiter(ref hipTargetPos, ref headTargetPos);

            var hipsSpineVecUpwards = math.mul(objective.hipTargetWorldRotation, math.right()) * hvr_godot_helper.GetX(definition.refPoseSpineVecForHipsRotation);
            var hipsSpineVecFrontwards = math.mul(objective.hipTargetWorldRotation, math.down()) * hvr_godot_helper.GetY(definition.refPoseSpineVecForHipsRotation);
            var headSpineVecUpwards = math.mul(objective.headTargetWorldRotation, math.right()) * hvr_godot_helper.GetX(definition.refPoseSpineVecForHeadRotation);
            var headSpineVecFrontwards = math.mul(objective.headTargetWorldRotation, math.down()) * hvr_godot_helper.GetY(definition.refPoseSpineVecForHeadRotation);

            // ## Try to fix spine buckle: Calculate spine tension to try fixing the buckle issue
            if (objective.improveSpineBuckling > 0f)
            {
                var effectiveDistanceAfterCorrections = math.distance(hipTargetPos, headTargetPos);
                var tension = 1 - effectiveDistanceAfterCorrections / (_hipsToSpineToCheckToNeckToHeadLength * scale);
                var tensionDirection = math.normalize(hipTargetPos - headTargetPos);
                var tensionVectorIsSimilarToSpineVector = math.clamp(math.unlerp(0.96f, 1f, math.dot(math.normalize(-hipsSpineVecUpwards), tensionDirection)), 0f, 1f);

                var totalTension = tension * tensionVectorIsSimilarToSpineVector * objective.improveSpineBuckling * scale;
                if (totalTension > 0f)
                {
                    var tensionVector = tensionDirection * totalTension;
                    var prevHipTargetPos = hipTargetPos;
                    hipTargetPos += tensionVector;
                    ApplyLimiter(ref hipTargetPos, ref headTargetPos);
                
#if UNITY_EDITOR && true
                    if (debugDrawSolver)
                    {
                        MbusUtil.DrawArrow(headTargetPos, headTargetPos + tensionVector, Color.cyan, 0f, false, math.mul(objective.headTargetWorldRotation, math.forward()));
                        MbusUtil.DrawArrow(prevHipTargetPos, prevHipTargetPos + tensionVector, Color.cyan, 0f, false, math.mul(objective.headTargetWorldRotation, math.forward()));
                    }
#endif
                }
            }

            // ## Prepare
            ikSnapshot.absolutePos[(int)Hips] = hipTargetPos;
            ikSnapshot.absoluteRot[(int)Hips] = objective.hipTargetWorldRotation;
            // The position of the spine is a direct consequence of the position and rotation of the hips, so we have this immediately,
            // and it won't change as a result of IK calculations until we get to the head alignment.
            ikSnapshot.ReevaluatePosition(Spine, definition, scale);
            var spinePos = ikSnapshot.absolutePos[(int)Spine];

            // ## Prime
            var spineToHead = headTargetPos - spinePos;
            var spineToHeadLen = math.length(spineToHead);
            
            var hipsSide = math.mul(objective.hipTargetWorldRotation, math.forward());
            
            var chestPosBase = spinePos + hipsSpineVecUpwards * spineToHeadLen * hvr_godot_helper.GetX(definition.refPoseChestRelation) 
                                        + hipsSpineVecFrontwards *  hvr_godot_helper.GetY(definition.refPoseChestRelation) * scale;
            var neckPosBase = headTargetPos - headSpineVecUpwards * spineToHeadLen * (1 -  hvr_godot_helper.GetX(definition.refPoseNeckRelation)) 
                              + headSpineVecFrontwards *  hvr_godot_helper.GetY(definition.refPoseNeckRelation) * scale;
            var useNeck = objective.useChest * objective.alsoUseChestToMoveNeck;
            var primingSpine = spinePos;
            var primingChest = objective.useChest <= 0 ? chestPosBase : math.lerp(chestPosBase, objective.chestTargetWorldPosition, objective.useChest);
            var primingNeck = useNeck <= 0 ? neckPosBase : math.lerp(neckPosBase, objective.chestTargetWorldPosition + math.mul(objective.chestTargetWorldRotation, math.right() * definition.refPoseChestLength * scale), useNeck);
            var primingHead = headTargetPos;
            _spineChain[0] = primingSpine; // Spine
            _spineChain[1] = primingChest; // Chest
            _spineChain[2] = primingNeck; // Neck
            _spineChain[3] = primingHead; // Head

            // Help with an issue with Hot Reload
            _spineChain.Length = 4;
            _spineDistances.Length = 3;
            
            // ## Relax
            var operationCounter = 0;
            for (var i = 0; i < objective.fabrikIterations; i++)
            {
                MbusMathSolver.Iterate(ref _spineChain, headTargetPos, _spineDistances, spinePos, ref operationCounter, Int32.MaxValue, scale);
#if UNITY_EDITOR && true
                if (debugDrawSolver)
                {
                    Debug.DrawLine(_spineChain[0], _spineChain[1], lawnGreenTransparent, 0f, false);
                    Debug.DrawLine(_spineChain[1], _spineChain[2], lawnGreenTransparent, 0f, false);
                    Debug.DrawLine(_spineChain[2], _spineChain[3], lawnGreenTransparent, 0f, false);
                }
#endif
            }

#if UNITY_EDITOR && true
            if (debugDrawSolver)
            {
                var arrowCross = math.normalize(hipsSide);
                MbusUtil.DrawArrow(_spineChain[0], primingSpine, lawnGreen, 0f, false, arrowCross);
                MbusUtil.DrawArrow(_spineChain[1], primingChest, lawnGreen, 0f, false, arrowCross);
                MbusUtil.DrawArrow(_spineChain[2], primingNeck, lawnGreen, 0f, false, arrowCross);
                MbusUtil.DrawArrow(_spineChain[3], primingHead, lawnGreen, 0f, false, arrowCross);
            }
#endif

            // ## Positions are solved into _spineChain. Now, solve the rotations.

            // These attempt to provide a proper roll for the extreme case when you're overbending: The head is pointing in opposite direction to the hip, e.g. near or more than 180 degrees
            // TODO: Replace this with a cross product
            var hipVec = SolveLerpVec(math.normalize(headTargetPos - hipTargetPos), objective.hipTargetWorldRotation);
            var headVec = SolveLerpVec(math.normalize(headTargetPos - hipTargetPos), objective.headTargetWorldRotation);
            var spineLerpVec = SolveLerpVec(math.normalize(_spineChain[1] - _spineChain[0]), objective.hipTargetWorldRotation);

            ikSnapshot.absoluteRot[(int)Hips] = objective.hipTargetWorldRotation;
            ikSnapshot.absoluteRot[(int)Spine] = math.mul(
                hvr_godot_helper_quaternion.LookRotationSafe(_spineChain[1] - _spineChain[0], spineLerpVec),
                _reorienter
            );
            var chestRotBase = math.normalize(MbusGeofunctions.Slerp(hipVec, headVec, 0.75f));
            ikSnapshot.absoluteRot[(int)Chest] = math.mul(
                hvr_godot_helper_quaternion.LookRotationSafe(_spineChain[2] - _spineChain[1], math.lerp(chestRotBase, math.mul(objective.chestTargetWorldRotation, math.down()), objective.useChest)),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)Neck] = math.mul(
                hvr_godot_helper_quaternion.LookRotationSafe(_spineChain[3] - _spineChain[2], math.mul(objective.headTargetWorldRotation, math.down())),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)Head] = objective.headTargetWorldRotation;

            // Recalculate the real position of the head, so that we may realign it.
            ikSnapshot.ReevaluatePosition(Spine, definition, scale);
            ikSnapshot.ReevaluatePosition(Chest, definition, scale);
            ikSnapshot.ReevaluatePosition(Neck, definition, scale);
            ikSnapshot.ReevaluatePosition(Head, definition, scale);
            
            // Realign the head, if applicable.
            if (objective.headAlignmentMattersMore)
            {
                var headMismatch2 = headTargetPos - ikSnapshot.absolutePos[(int)Head];
                ikSnapshot.absolutePos[(int)Hips] += headMismatch2;
                ikSnapshot.ReevaluatePosition(Spine, definition, scale);
                ikSnapshot.ReevaluatePosition(Chest, definition, scale);
                ikSnapshot.ReevaluatePosition(Neck, definition, scale);
                ikSnapshot.ReevaluatePosition(Head, definition, scale);
            }
            
            // Recalculate the shoulders, arms and legs, so that we may calculate the arms next
            ikSnapshot.ReevaluatePosition(LeftShoulder, definition, scale);
            ikSnapshot.ReevaluatePosition(RightShoulder, definition, scale);
            
            ikSnapshot.ApplyReferenceRotation(LeftShoulder, definition);
            ikSnapshot.ApplyReferenceRotation(RightShoulder, definition);
            
            ikSnapshot.ReevaluatePosition(LeftUpperArm, definition, scale);
            ikSnapshot.ReevaluatePosition(RightUpperArm, definition, scale);
            ikSnapshot.ReevaluatePosition(LeftUpperLeg, definition, scale);
            ikSnapshot.ReevaluatePosition(RightUpperLeg, definition, scale);
            
#if UNITY_EDITOR && true
            if (debugDrawSolver)
            {
                Debug.DrawLine(ikSnapshot.absolutePos[(int)Hips], hipTargetPos, Color.magenta, 0f, false);
            
                Debug.DrawLine(ikSnapshot.absolutePos[(int)Chest], ikSnapshot.absolutePos[(int)LeftShoulder], coral, 0f, false);
                Debug.DrawLine(ikSnapshot.absolutePos[(int)Chest], ikSnapshot.absolutePos[(int)RightShoulder], coral, 0f, false);
                Debug.DrawLine(ikSnapshot.absolutePos[(int)LeftShoulder], ikSnapshot.absolutePos[(int)LeftUpperArm], coral, 0f, false);
                Debug.DrawLine(ikSnapshot.absolutePos[(int)RightShoulder], ikSnapshot.absolutePos[(int)RightUpperArm], coral, 0f, false);
                Debug.DrawLine(ikSnapshot.absolutePos[(int)Hips], ikSnapshot.absolutePos[(int)Spine], coral, 0f, false);
                Debug.DrawLine(ikSnapshot.absolutePos[(int)Hips], ikSnapshot.absolutePos[(int)LeftUpperLeg], coral, 0f, false);
                Debug.DrawLine(ikSnapshot.absolutePos[(int)Hips], ikSnapshot.absolutePos[(int)RightUpperLeg], coral, 0f, false);
            
                if (objective.useChest > 0f)
                {
                    Debug.DrawLine(ikSnapshot.absolutePos[(int)Chest], objective.chestTargetWorldPosition, mediumOrchid, 0f, false);
                }
            }
#endif

            return ikSnapshot;
        }

        private static float3 SolveLerpVec(float3 similarityVector, quaternion ikRot)
        {
            var regular = math.mul(ikRot, math.down());
            var ifOne = math.mul(ikRot, math.left());
            var ifMinusOne = math.mul(ikRot, math.right());

            var dot = math.dot(similarityVector, regular);
            
            return MbusGeofunctions.LerpDot(ifMinusOne, regular, ifOne, dot);
        }
    }
}