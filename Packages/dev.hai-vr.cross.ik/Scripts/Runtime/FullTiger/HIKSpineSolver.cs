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
        private readonly HIKSolver solver;

        private readonly float3[] _spineChain;
        private float[] _spineDistances;

        public HIKSpineSolver(HIKAvatarDefinition definition, HIKSnapshot ikSnapshot, quaternion reorienter, HIKSolver solver)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");
            
            this.definition = definition;
            this.ikSnapshot = ikSnapshot;
            _reorienter = reorienter;
            this.solver = solver;

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

            var ff = math.lerp(0.01f, 0.7f, headAndHipSameDirection01);
            if (math.distance(originalHipTargetPos, headTargetPos) < definition.refPoseHipToHeadLength * ff) // TODO: Allow this to be closer if the head and hip are not in the same direction
            {
                hipTargetPos = headTargetPos - math.normalize(headTargetPos - originalHipTargetPos) * definition.refPoseHipToHeadLength * ff;
                Debug.DrawLine(headTargetPos, hipTargetPos, Color.yellow, 0f, false);
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

            // Reset snapshot
            ikSnapshot.absolutePos[(int)Hips] = hipTargetPos;
            for (var boneId = Hips; boneId <= LeftEye; boneId++)
            {
                solver.SetSnapshotToReferencePose(boneId == LeftEye ? UpperChest : boneId);
            }

            // Prepare
            ikSnapshot.absoluteRot[(int)Hips] = objective.hipTargetWorldRotation;
            ikSnapshot.ReevaluatePosition(Spine, definition);
            var spinePos = ikSnapshot.absolutePos[(int)Spine];
            
            var spineToHead = headTargetPos - spinePos;

            // Prime
            var back = math.mul(objective.headTargetWorldRotation, math.up());
            var spintToHeadLen = math.length(spineToHead);
            
            // TODO: We should prime the spine based on what the reference pose already suggested.
            _spineChain[0] = spinePos; // Spine
            _spineChain[1] = spinePos + math.mul(objective.hipTargetWorldRotation, math.right()) * spintToHeadLen * 0.3f + back * 0.05f; // Chest
            _spineChain[2] = headTargetPos - math.mul(objective.headTargetWorldRotation, math.right()) * spintToHeadLen * 0.3f + back * 0.05f; // Neck
            _spineChain[3] = headTargetPos; // Head
            
            ikSnapshot.ReevaluatePosition(Chest, definition);
            ikSnapshot.ReevaluatePosition(Neck, definition);
            ikSnapshot.ReevaluatePosition(Head, definition);
            
            // Relax
            var operationCounter = 0;
            for (var i = 0; i < Iterations; i++)
            {
                // FIXME: This solver overextends beyond definition.refPoseHipToNeckLength. We should take that into account.
                MbusMathSolver.Iterate(_spineChain, headTargetPos, _spineDistances, spinePos, ref operationCounter, Int32.MaxValue);
                // var color = Color.Lerp(Color.black, Color.red, i / (Iterations - 1f));
                // if (drawDebug) DataViz.Instance.DrawLine(spineBezier, color, color);
            }

            // We don't need to run this because it's recalculated by ReevaluatePosition later.
            if (false)
            {
                ikSnapshot.absolutePos[(int)Spine] = _spineChain[0];
                ikSnapshot.absolutePos[(int)Chest] = _spineChain[1];
                ikSnapshot.absolutePos[(int)Neck] = _spineChain[2];
                ikSnapshot.absolutePos[(int)Head] = _spineChain[3];
            }
            
            // Positions are solved. Now, solve the rotations.
            
            ikSnapshot.absoluteRot[(int)Hips] = objective.hipTargetWorldRotation;
            ikSnapshot.absoluteRot[(int)Spine] = math.mul(
                quaternion.LookRotationSafe(_spineChain[1] - _spineChain[0], math.mul(objective.hipTargetWorldRotation, math.down())),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)Chest] = math.mul(
                quaternion.LookRotationSafe(_spineChain[2] - _spineChain[1], math.mul(math.slerp(objective.hipTargetWorldRotation, objective.headTargetWorldRotation, 0.75f), math.down())),
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
            }
        }
    }
}