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
using Unity.Mathematics;
using UnityEngine;
using static UnityEngine.HumanBodyBones;

namespace HVR.IK.FullTiger
{
    internal class HIKSpineSolver
    {
        private const int Iterations = 10;
        
        private readonly HIKAvatarDefinition definition;
        private readonly HIKSnapshot ikSnapshot;
        private readonly quaternion _reorienter;

        private readonly float3[] _spineChain;
        private float[] _spineDistances;

        public HIKSpineSolver(HIKAvatarDefinition definition, HIKSnapshot ikSnapshot, quaternion reorienter)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");
            
            this.definition = definition;
            this.ikSnapshot = ikSnapshot;
            _reorienter = reorienter;

            _spineChain = new float3[4];
            _spineDistances = new[]
            {
                math.distance(definition.refPoseHiplativePos[(int)Spine], definition.refPoseHiplativePos[(int)Chest]), 
                math.distance(definition.refPoseHiplativePos[(int)Chest], definition.refPoseHiplativePos[(int)Neck]), 
                math.distance(definition.refPoseHiplativePos[(int)Neck], definition.refPoseHiplativePos[(int)Head]), 
            };
        }

        public void Solve(HIKObjective objective)
        {
            var originalHipTargetPos = objective.hipTargetWorldPosition;
            var originalHeadTargetPos = objective.headTargetWorldPosition;
            
            var hipTargetPos = originalHipTargetPos;
            var headTargetPos = originalHeadTargetPos;
            
            var headAndHipSameDirection01 = math.dot(math.mul(objective.headTargetWorldRotation, math.right()), math.mul(objective.hipTargetWorldRotation, math.right())) * 0.5f + 0.5f;

            if (!objective.allowContortionist)
            {
                var ff = math.lerp(0.01f, (definition.refPoseHipToNeckLength + definition.refPoseNeckLength * 0.75f) / definition.refPoseHipToHeadLength, headAndHipSameDirection01);
                if (math.distance(originalHipTargetPos, headTargetPos) < definition.refPoseHipToHeadLength * ff) // TODO: Allow this to be closer if the head and hip are not in the same direction
                {
                    hipTargetPos = headTargetPos - math.normalize(headTargetPos - originalHipTargetPos) * definition.refPoseHipToHeadLength * ff;
                    Debug.DrawLine(headTargetPos, hipTargetPos, Color.red, 0f, false);
                }
                else
                {
                    var kk = math.normalize(headTargetPos - hipTargetPos) * definition.refPoseHipToHeadLength * ff;
                    Debug.DrawLine(hipTargetPos, hipTargetPos + kk, Color.yellow, 0f, false);
                }
            }
            
            // If the distance between the head and the neck is larger than the length of the neck + refPoseHipToNeckLength
            // (which is not equal to the sum of the bones of the hip-spine-chest-neck) chain, then either the head or the hips MUST be brought closer
            // so that the solver doesn't overstretch the artists' spine.
            var refHipToNeckAndThenToHeadLength = definition.refPoseHipToNeckLength + definition.refPoseNeckLength;
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

            // ## Prepare
            ikSnapshot.absolutePos[(int)Hips] = hipTargetPos;
            ikSnapshot.absoluteRot[(int)Hips] = objective.hipTargetWorldRotation;
            // The position of the spine is a direct consequence of the position and rotation of the hips, so we have this immediately,
            // and it won't change as a result of IK calculations until we get to the head alignment.
            ikSnapshot.ReevaluatePosition(Spine, definition);
            var spinePos = ikSnapshot.absolutePos[(int)Spine];

            // ## Prime
            var spineToHead = headTargetPos - spinePos;
            var back = math.mul(objective.headTargetWorldRotation, math.down());
            var spintToHeadLen = math.length(spineToHead);
        
            // TODO: We should prime the spine based on what the reference pose already suggested.
            _spineChain[0] = spinePos; // Spine
            var chestPosBase = spinePos + math.mul(objective.hipTargetWorldRotation, math.right()) * spintToHeadLen * 0.3f + back * 0.01f;
            _spineChain[1] = math.lerp(chestPosBase, objective.chestTargetWorldPosition, objective.useChest); // Chest
            var neckPosBase = headTargetPos - math.mul(objective.headTargetWorldRotation, math.right()) * spintToHeadLen * 0.3f + back * 0.01f;
            _spineChain[2] = math.lerp(neckPosBase, objective.chestTargetWorldPosition + math.mul(objective.chestTargetWorldRotation, math.right() * spintToHeadLen * 0.4f), objective.useChest * objective.alsoUseChestToMoveNeck); // Neck
            _spineChain[3] = headTargetPos; // Head
            
            // ## Relax
            var operationCounter = 0;
            for (var i = 0; i < Iterations; i++)
            {
                MbusMathSolver.Iterate(_spineChain, headTargetPos, _spineDistances, spinePos, ref operationCounter, Int32.MaxValue);
                // var color = Color.Lerp(Color.black, Color.red, i / (Iterations - 1f));
                // if (drawDebug) DataViz.Instance.DrawLine(spineBezier, color, color);
            }

            // ## Positions are solved into _spineChain. Now, solve the rotations.

            // These attempt to provide a proper roll for the extreme case when you're overbending: The head is pointing in opposite direction to the hip, e.g. near or more than 180 degrees
            var hipVec = SolveLerpVec(math.normalize(headTargetPos - hipTargetPos), objective.hipTargetWorldRotation);
            var headVec = SolveLerpVec(math.normalize(headTargetPos - hipTargetPos), objective.headTargetWorldRotation);
            var spineLerpVec = SolveLerpVec(math.normalize(_spineChain[1] - _spineChain[0]), objective.hipTargetWorldRotation);

            ikSnapshot.absoluteRot[(int)Hips] = objective.hipTargetWorldRotation;
            ikSnapshot.absoluteRot[(int)Spine] = math.mul(
                quaternion.LookRotationSafe(_spineChain[1] - _spineChain[0], spineLerpVec),
                _reorienter
            );
            var chestRotBase = math.normalize(Vector3.Slerp(hipVec, headVec, 0.75f));
            ikSnapshot.absoluteRot[(int)Chest] = math.mul(
                // FIXME: Vector3.Slerp doesn't use unity mathematics.
                quaternion.LookRotationSafe(_spineChain[2] - _spineChain[1], math.lerp(chestRotBase, math.mul(objective.chestTargetWorldRotation, math.down()), objective.useChest)),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)Neck] = math.mul(
                quaternion.LookRotationSafe(_spineChain[3] - _spineChain[2], math.mul(objective.headTargetWorldRotation, math.down())),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)Head] = objective.headTargetWorldRotation;

            if (true)
            {
                // Recalculate the real position of the head, so that we may realign it.
                ikSnapshot.ReevaluatePosition(Spine, definition);
                ikSnapshot.ReevaluatePosition(Chest, definition);
                ikSnapshot.ReevaluatePosition(Neck, definition);
                ikSnapshot.ReevaluatePosition(Head, definition);
                
                // Realign the head, if applicable.
                if (objective.headAlignmentMattersMore)
                {
                    var headMismatch2 = headTargetPos - ikSnapshot.absolutePos[(int)Head];
                    ikSnapshot.absolutePos[(int)Hips] += headMismatch2;
                    ikSnapshot.ReevaluatePosition(Spine, definition);
                    ikSnapshot.ReevaluatePosition(Chest, definition);
                    ikSnapshot.ReevaluatePosition(Neck, definition);
                    ikSnapshot.ReevaluatePosition(Head, definition);
                }
                
                // Recalculate the shoulders, arms and legs, so that we may calculate the arms next
                ikSnapshot.ReevaluatePosition(LeftShoulder, definition);
                ikSnapshot.ReevaluatePosition(RightShoulder, definition);
                
                ikSnapshot.ApplyReferenceRotation(LeftShoulder, definition);
                ikSnapshot.ApplyReferenceRotation(RightShoulder, definition);
                
                ikSnapshot.ReevaluatePosition(LeftUpperArm, definition);
                ikSnapshot.ReevaluatePosition(RightUpperArm, definition);
                ikSnapshot.ReevaluatePosition(LeftUpperLeg, definition);
                ikSnapshot.ReevaluatePosition(RightUpperLeg, definition);
                Debug.DrawLine(ikSnapshot.absolutePos[(int)Hips], hipTargetPos, Color.magenta, 0f, false);
                
                Debug.DrawLine(ikSnapshot.absolutePos[(int)Chest], ikSnapshot.absolutePos[(int)LeftShoulder], Color.coral, 0f, false);
                Debug.DrawLine(ikSnapshot.absolutePos[(int)Chest], ikSnapshot.absolutePos[(int)RightShoulder], Color.coral, 0f, false);
                Debug.DrawLine(ikSnapshot.absolutePos[(int)LeftShoulder], ikSnapshot.absolutePos[(int)LeftUpperArm], Color.coral, 0f, false);
                Debug.DrawLine(ikSnapshot.absolutePos[(int)RightShoulder], ikSnapshot.absolutePos[(int)RightUpperArm], Color.coral, 0f, false);
                Debug.DrawLine(ikSnapshot.absolutePos[(int)Hips], ikSnapshot.absolutePos[(int)LeftUpperLeg], Color.coral, 0f, false);
                Debug.DrawLine(ikSnapshot.absolutePos[(int)Hips], ikSnapshot.absolutePos[(int)RightUpperLeg], Color.coral, 0f, false);

                if (objective.useChest > 0f)
                {
                    Debug.DrawLine(ikSnapshot.absolutePos[(int)Chest], objective.chestTargetWorldPosition, Color.mediumOrchid, 0f, false);
                }
            }
        }

        private static float3 SolveLerpVec(float3 similarityVector, quaternion ikRot)
        {
            var regular = math.mul(ikRot, math.down());
            var ifOne = math.mul(ikRot, math.left());
            var ifMinusOne = math.mul(ikRot, math.right());

            var dot = math.dot(similarityVector, regular);
            
            return MbusUtil.LerpDot(ifMinusOne, regular, ifOne, dot);
        }
    }
}