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
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using static HVR.IK.FullTiger.HIKBodyBones;

namespace HVR.IK.FullTiger
{
    internal struct HIKSpineSolver : IDisposable
    {
        private const int Iterations = 10;
        
        private readonly HIKAvatarDefinition definition;
        private readonly quaternion _reorienter;

        private NativeArray<float3> _spineChain;
        private NativeArray<float> _spineDistances;
        
        private static readonly Color lawnGreen = new Color(0.4862745f, 0.9882354f, 0.0f, 1f);
        private static readonly Color coral = new Color(1f, 0.4980392f, 0.3137255f, 1f);
        private static readonly Color mediumOrchid = new Color(0.7294118f, 0.3333333f, 0.8274511f, 1f);

        public HIKSpineSolver(HIKAvatarDefinition definition, quaternion reorienter)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");
            
            this.definition = definition;
            _reorienter = reorienter;

            _spineChain = new NativeArray<float3>(4, Allocator.Persistent);
            _spineDistances = new NativeArray<float>(3, Allocator.Persistent);
            _spineDistances[0] = math.distance(definition.refPoseHiplativePos[(int)Spine], definition.refPoseHiplativePos[(int)Chest]);
            _spineDistances[1] = math.distance(definition.refPoseHiplativePos[(int)Chest], definition.refPoseHiplativePos[(int)Neck]);
            _spineDistances[2] = math.distance(definition.refPoseHiplativePos[(int)Neck], definition.refPoseHiplativePos[(int)Head]);
        }

        public void Dispose()
        {
            if (_spineChain.IsCreated) _spineChain.Dispose();
            if (_spineDistances.IsCreated) _spineDistances.Dispose();
        }

        public HIKSnapshot Solve(HIKObjective objective, HIKSnapshot ikSnapshot)
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
                    Debug.DrawLine(headTargetPos, hipTargetPos, Color.red, 0f, false);
#endif
                }
                else
                {
                    var kk = math.normalize(headTargetPos - hipTargetPos) * definition.refPoseHipToHeadLength * ff;
#if UNITY_EDITOR && true
                    Debug.DrawLine(hipTargetPos, hipTargetPos + kk, Color.yellow, 0f, false);
#endif
                }
            }

            if (!objective.doNotPreserveHipsToNeckCurvatureLimit)
            {
                // If the distance between the head and the neck is larger than the length of the neck + refPoseHipToNeckLength
                // (which is not equal to the sum of the bones of the hip-spine-chest-neck) chain, then either the head or the hips MUST be brought closer
                // so that the solver doesn't overstretch the artists' spine.
                var refHipToNeckAndThenToHeadLength = (definition.refPoseHipToNeckLength + definition.refPoseNeckLength) * scale;
                if (math.distance(hipTargetPos, headTargetPos) > refHipToNeckAndThenToHeadLength)
                {
                    if (objective.headAlignmentMattersMore)
                    {
                        var toHip = math.normalize(hipTargetPos - headTargetPos);
                        hipTargetPos = headTargetPos + toHip * refHipToNeckAndThenToHeadLength;
                    }
                    else
                    {
                        var toHead = math.normalize(headTargetPos - hipTargetPos);
                        headTargetPos = hipTargetPos + toHead * refHipToNeckAndThenToHeadLength;
                    }
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
            var back = math.mul(objective.headTargetWorldRotation, math.down());
            var spineToHeadLen = math.length(spineToHead);
            // var spineToHeadNormalized = math.normalize(spineToHead);
        
            // TODO: We should prime the spine based on what the reference pose already suggested.
            var chestPosBase = spinePos + math.mul(objective.hipTargetWorldRotation, math.right()) * spineToHeadLen * 0.3f + back * 0.01f * scale;
            var neckPosBase = headTargetPos - math.mul(objective.headTargetWorldRotation, math.right()) * spineToHeadLen * 0.3f + back * 0.01f * scale;
            // var chestPosBase = spinePos + spineToHeadNormalized * definition.refPoseChestLength * scale + back * 0.01f * scale;
            // var neckPosBase = headTargetPos - spineToHeadNormalized * definition.refPoseNeckLength * scale + back * 0.01f * scale;
            var primingSpine = spinePos;
            var primingChest = math.lerp(chestPosBase, objective.chestTargetWorldPosition, objective.useChest);
            var primingNeck = math.lerp(neckPosBase, objective.chestTargetWorldPosition + math.mul(objective.chestTargetWorldRotation, math.right() * definition.refPoseChestLength * scale), objective.useChest * objective.alsoUseChestToMoveNeck);
            var primingHead = headTargetPos;
            _spineChain[0] = primingSpine; // Spine
            _spineChain[1] = primingChest; // Chest
            _spineChain[2] = primingNeck; // Neck
            _spineChain[3] = primingHead; // Head
            
            // ## Relax
            var operationCounter = 0;
            for (var i = 0; i < Iterations; i++)
            {
                MbusMathSolver.Iterate(_spineChain, headTargetPos, _spineDistances, spinePos, ref operationCounter, Int32.MaxValue, scale);
                // var color = Color.Lerp(Color.black, Color.red, i / (Iterations - 1f));
                // if (drawDebug) DataViz.Instance.DrawLine(spineBezier, color, color);
            }

#if UNITY_EDITOR && true
            Debug.DrawLine(primingSpine, _spineChain[0], lawnGreen, 0f, false);
            Debug.DrawLine(primingChest, _spineChain[1], lawnGreen, 0f, false);
            Debug.DrawLine(primingNeck, _spineChain[2], lawnGreen, 0f, false);
            Debug.DrawLine(primingHead, _spineChain[3], lawnGreen, 0f, false);
#endif

            // ## Positions are solved into _spineChain. Now, solve the rotations.

            // These attempt to provide a proper roll for the extreme case when you're overbending: The head is pointing in opposite direction to the hip, e.g. near or more than 180 degrees
            // TODO: Replace this with a cross product
            var hipVec = SolveLerpVec(math.normalize(headTargetPos - hipTargetPos), objective.hipTargetWorldRotation);
            var headVec = SolveLerpVec(math.normalize(headTargetPos - hipTargetPos), objective.headTargetWorldRotation);
            var spineLerpVec = SolveLerpVec(math.normalize(_spineChain[1] - _spineChain[0]), objective.hipTargetWorldRotation);

            ikSnapshot.absoluteRot[(int)Hips] = objective.hipTargetWorldRotation;
            ikSnapshot.absoluteRot[(int)Spine] = math.mul(
                quaternion.LookRotationSafe(_spineChain[1] - _spineChain[0], spineLerpVec),
                _reorienter
            );
            var chestRotBase = math.normalize(MbusGeofunctions.Slerp(hipVec, headVec, 0.75f));
            ikSnapshot.absoluteRot[(int)Chest] = math.mul(
                quaternion.LookRotationSafe(_spineChain[2] - _spineChain[1], math.lerp(chestRotBase, math.mul(objective.chestTargetWorldRotation, math.down()), objective.useChest)),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)Neck] = math.mul(
                quaternion.LookRotationSafe(_spineChain[3] - _spineChain[2], math.mul(objective.headTargetWorldRotation, math.down())),
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
            Debug.DrawLine(ikSnapshot.absolutePos[(int)Hips], hipTargetPos, Color.magenta, 0f, false);
            
            Debug.DrawLine(ikSnapshot.absolutePos[(int)Chest], ikSnapshot.absolutePos[(int)LeftShoulder], coral, 0f, false);
            Debug.DrawLine(ikSnapshot.absolutePos[(int)Chest], ikSnapshot.absolutePos[(int)RightShoulder], coral, 0f, false);
            Debug.DrawLine(ikSnapshot.absolutePos[(int)LeftShoulder], ikSnapshot.absolutePos[(int)LeftUpperArm], coral, 0f, false);
            Debug.DrawLine(ikSnapshot.absolutePos[(int)RightShoulder], ikSnapshot.absolutePos[(int)RightUpperArm], coral, 0f, false);
            Debug.DrawLine(ikSnapshot.absolutePos[(int)Hips], ikSnapshot.absolutePos[(int)LeftUpperLeg], coral, 0f, false);
            Debug.DrawLine(ikSnapshot.absolutePos[(int)Hips], ikSnapshot.absolutePos[(int)RightUpperLeg], coral, 0f, false);
            
            if (objective.useChest > 0f)
            {
                Debug.DrawLine(ikSnapshot.absolutePos[(int)Chest], objective.chestTargetWorldPosition, mediumOrchid, 0f, false);
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