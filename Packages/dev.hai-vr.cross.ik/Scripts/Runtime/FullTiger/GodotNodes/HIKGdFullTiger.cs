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

#if !UNITY_2022_1_OR_NEWER //__GODOT
using Godot;
using HVR.IK.FullTiger;
using static HVR.IK.FullTiger.HIKBodyBones;

namespace HVR.IK.Gd.FullTiger;

public partial class HIKGdFullTiger : Node
{
	private static readonly HIKBodyBones[] AcceptableBodyBones =
	{
		Hips, Spine, Chest, UpperChest, Neck, Head,
		LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand,
		RightShoulder, RightUpperArm, RightLowerArm, RightHand,
		LeftUpperLeg, LeftLowerLeg, LeftFoot, LeftToes,
		RightUpperLeg, RightLowerLeg, RightFoot, RightToes
	};
	private static readonly HIKBodyBones[] CopyOrder =
	{
		/*Hips,*/ Spine, Chest, UpperChest, Neck, Head,
		LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand,
		RightShoulder, RightUpperArm, RightLowerArm, RightHand,
		LeftUpperLeg, LeftLowerLeg, LeftFoot, //LeftToes,
		RightUpperLeg, RightLowerLeg, RightFoot, //RightToes,
	};
	private static readonly HIKBodyBones[] CopyOrderSpineAndShoulders =
	{
		/*Hips,*/ Spine, Chest, UpperChest, Neck, Head,
		LeftShoulder,
		RightShoulder
	};
	private static readonly HIKBodyBones[] CopyOrderLeftArm =
	{
		LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand,
	};
	private static readonly HIKBodyBones[] CopyOrderRightArm =
	{
		RightShoulder, RightUpperArm, RightLowerArm, RightHand,
	};
	private static readonly HIKBodyBones[] CopyOrderLeftLeg =
	{
		LeftUpperLeg, LeftLowerLeg, LeftFoot, //LeftToes,
	};
	private static readonly HIKBodyBones[] CopyOrderRightLeg =
	{
		RightUpperLeg, RightLowerLeg, RightFoot, //RightToes,
	};
	private static readonly HIKBodyBones[] SpineChain = { Hips, Spine, Chest, UpperChest, Neck, Head };
	private static readonly HIKBodyBones[] LeftArmChain = { LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand };
	private static readonly HIKBodyBones[] RightArmChain = { RightShoulder, RightUpperArm, RightLowerArm, RightHand };
	
	[Export] public Skeleton3D Skeleton { get; set; }
	[Export] public Node3D SkeletonSearch { get; set; }
	[Export] public HIKGdEffectors Effectors { get; set; }

	[ExportGroup("Solver Settings")]
	[Export] public bool OverrideDefaultFabrikIterationCount { get; set; } = false;
	[Export] public int FabrikIterations { get; set; } = HIKSpineSolver.Iterations;

	[ExportGroup("Debug")]
	[Export] public bool DebugDrawFinalChains { get; set; } = true;
	[Export] public bool DebugDrawSolver { get; private set; } = true;

	private HIKGdHumanoidMapper _humanoid;
	private HIKAvatarDefinition definition = new();
	private HIKSolver _ikSolver;
	private HIKSnapshot _ikSnapshot = new();
		
	private bool _solveSpine = true;
	private bool _solveLeftLeg = true;
	private bool _solveRightLeg = true;
	private bool _solveLeftArm = true;
	private bool _solveRightArm = true;
	
	private bool _useFakeDoubleJointedKneesWasEverEnabled;

	public override void _Ready()
	{
		if (SkeletonSearch != null && Skeleton == null) Skeleton = HIKGdUtil.FindFirstNodeWithScript<Skeleton3D>(SkeletonSearch);

		if (Skeleton == null)
		{
			GD.PrintErr("HIKGdFullTiger: No Skeleton3D found");
			return;
		}

		_humanoid = new HIKGdHumanoidMapper(Skeleton);
		
		definition = SolveDefinition(_humanoid, new HIKAvatarDefinition());
			
		// Order matters: This must be instantiated AFTER definition is initialized
		_ikSolver = new HIKSolver(definition);

		DebugDraw3D.ScopedConfig().SetThickness(0.005f).SetNoDepthTest(true);
	}

	internal static HIKAvatarDefinition SolveDefinition(HIKGdHumanoidMapper humanoidMapper, HIKAvatarDefinition definition)
	{
		var inverseOfWorldSpaceHipsRotation = hvr_godot_math.inverse(humanoidMapper.GetWorldSpaceHipsRotation());
		
		// TODO: We should T-Pose the avatar before sampling the hiplative positions
		var hips = humanoidMapper.GetBoneTransformAtRest(Hips);
		foreach (var boneId in AcceptableBodyBones)
		{
			var i = (int)boneId;
			if (humanoidMapper.HasBone(boneId))
			{
				// var bone = humanoidMapper.GetBoneTransformAtRest(boneId);
				var postRot = humanoidMapper.GetPostRotation(boneId);

				definition.artistPosePos[i] = humanoidMapper.GetLocalPosition(boneId);
				definition.artistPoseRot[i] = humanoidMapper.GetLocalRotation(boneId);

				definition.refPoseRelativePos[i] = humanoidMapper.GetLocalPosition(boneId); // TODO
				definition.refPoseRelativeRot[i] = humanoidMapper.GetLocalRotation(boneId); // TODO
				// FIXME: Not sure if hips.Inverse() will work here (what coordinate space is hips in?)
				definition.refPoseHiplativePos[i] = hips.Inverse() * humanoidMapper.GetWorldSpacePosition(boneId); // hips.InverseTransformPoint(bone.position)
				definition.refPoseHiplativeRot[i] = inverseOfWorldSpaceHipsRotation * humanoidMapper.GetWorldSpaceRotation(boneId); // TODO

				definition.dataHasBone[i] = true;
				definition.dataPostRot[i] = postRot;
				definition.dataInversePostRot[i] = hvr_godot_math.inverse(postRot);

				definition.relativeMatrices[i] = hvr_godot_helper.float4x4_TRS(humanoidMapper.GetLocalPosition(boneId), humanoidMapper.GetLocalRotation(boneId), humanoidMapper.GetLocalScale(boneId));
			}
			else
			{
				definition.artistPosePos[i] = Vector3.Zero;
				definition.artistPoseRot[i] = Quaternion.Identity;

				definition.refPoseRelativePos[i] = Vector3.Zero;
				definition.refPoseRelativeRot[i] = Quaternion.Identity;
				definition.refPoseHiplativePos[i] = Vector3.Zero;
				definition.refPoseHiplativeRot[i] = Quaternion.Identity;

				definition.dataHasBone[i] = false;
				definition.dataPostRot[i] = Quaternion.Identity;
				definition.dataInversePostRot[i] = Quaternion.Identity;

				definition.relativeMatrices[i] = Transform3D.Identity;
			}
		}

		definition.refPoseHipToNeckLength = hvr_godot_math.distance(definition.refPoseHiplativePos[(int)Hips], definition.refPoseHiplativePos[(int)Neck]);
		definition.refPoseHipToHeadLength = hvr_godot_math.distance(definition.refPoseHiplativePos[(int)Hips], definition.refPoseHiplativePos[(int)Head]);

		definition.refPoseHipsLength = hvr_godot_math.distance(definition.refPoseHiplativePos[(int)Hips], definition.refPoseHiplativePos[(int)Spine]);
		definition.refPoseSpineLength = hvr_godot_math.distance(definition.refPoseHiplativePos[(int)Spine], definition.refPoseHiplativePos[(int)Chest]);
		definition.refPoseChestLength = hvr_godot_math.distance(definition.refPoseHiplativePos[(int)Chest], definition.refPoseHiplativePos[(int)Neck]);
		definition.refPoseNeckLength = hvr_godot_math.distance(definition.refPoseHiplativePos[(int)Neck], definition.refPoseHiplativePos[(int)Head]);

		{
			// Encode the spine curvature

			// var spine = humanoidMapper.GetBoneTransformAtRest(Spine);
			// var chest = humanoidMapper.GetBoneTransformAtRest(Chest);
			// var neck = humanoidMapper.GetBoneTransformAtRest(Neck);
			// var head = humanoidMapper.GetBoneTransformAtRest(Head);

			var hipsRef = hvr_godot_math.mul(humanoidMapper.GetWorldSpaceRotation(Hips), definition.dataPostRot[(int)Hips]);
			var headRef = hvr_godot_math.mul(humanoidMapper.GetWorldSpaceRotation(Head), definition.dataPostRot[(int)Head]);

			var spineToHead = (Vector3)(humanoidMapper.GetWorldSpacePosition(Head) - humanoidMapper.GetWorldSpacePosition(Spine));
			var spineToHeadLength = hvr_godot_math.length(spineToHead);
			var spineToHeadNormalized = hvr_godot_math.normalize(spineToHead);

			var spineToChest = (Vector3)(humanoidMapper.GetWorldSpacePosition(Chest) - humanoidMapper.GetWorldSpacePosition(Spine));
			var spineToNeck = (Vector3)(humanoidMapper.GetWorldSpacePosition(Neck) - humanoidMapper.GetWorldSpacePosition(Spine));

			var hipsSide = hvr_godot_math.mul(hipsRef, hvr_godot_math.forward());
			var crossNormalized = hvr_godot_math.normalize(hvr_godot_math.cross(spineToHeadNormalized, hipsSide));

			definition.refPoseChestRelation = new Vector2(hvr_godot_math.dot(spineToHeadNormalized, spineToChest) / spineToHeadLength, hvr_godot_math.dot(crossNormalized, spineToChest));
			definition.refPoseNeckRelation = new Vector2(hvr_godot_math.dot(spineToHeadNormalized, spineToNeck) / spineToHeadLength, hvr_godot_math.dot(crossNormalized, spineToNeck));

			definition.refPoseSpineVecForHipsRotation = new Vector2(hvr_godot_math.dot(hvr_godot_math.mul(hipsRef, hvr_godot_math.right()), spineToHeadNormalized), hvr_godot_math.dot(hvr_godot_math.mul(hipsRef, hvr_godot_math.down()), spineToHeadNormalized));
			definition.refPoseSpineVecForHeadRotation = new Vector2(hvr_godot_math.dot(hvr_godot_math.mul(headRef, hvr_godot_math.right()), spineToHeadNormalized), hvr_godot_math.dot(hvr_godot_math.mul(headRef, hvr_godot_math.down()), spineToHeadNormalized));
		}

		definition.capturedWithLossyScale = humanoidMapper.ResolveLossyScale();
		definition.isInitialized = true;

		return definition;
	}
	
	public override void _Process(double delta)
	{
		PerformRegularSolve();
		ApplySnapshot();

		if (DebugDrawFinalChains)
		{
			DrawArmChain(SpineChain);
			DrawArmChain(LeftArmChain);
			DrawArmChain(RightArmChain);
		}
	}

	public void PerformRegularSolve()
	{
		HIKSelfParenting selfParentLeftHand;
		if (Effectors.UseSelfParentLeftHand > 0f)
		{
			selfParentLeftHand = new HIKSelfParenting
			{
				use = Effectors.UseSelfParentLeftHand,
				bone = (HIKBodyBones)(int)Effectors.SelfParentLeftHandBone,
				relPosition = Effectors.SelfParentLeftHandRelativePosition,
				relRotation = Quaternion.FromEuler(Effectors.SelfParentLeftHandRelativeRotationEuler),
			};
		}
		else
		{
			selfParentLeftHand = default;
		}
		HIKSelfParenting selfParentRightHand;
		if (Effectors.UseSelfParentRightHand > 0f)
		{
			selfParentRightHand = new HIKSelfParenting
			{
				use = Effectors.UseSelfParentRightHand,
				bone = (HIKBodyBones)(int)Effectors.SelfParentRightHandBone,
				relPosition = Effectors.SelfParentRightHandRelativePosition,
				relRotation = Quaternion.FromEuler(Effectors.SelfParentRightHandRelativeRotationEuler),
			};
		}
		else
		{
			selfParentRightHand = default;
		}
		
		_ikSnapshot = _ikSolver.Solve(new HIKObjective
		{
			hipTargetWorldPosition = Effectors.HipTarget.GlobalPosition,
			hipTargetWorldRotation = Quaternion.FromEuler(Effectors.HipTarget.GlobalRotation),
			
			headTargetWorldPosition = Effectors.HeadTarget.GlobalPosition,
			headTargetWorldRotation = Quaternion.FromEuler(Effectors.HeadTarget.GlobalRotation),
			
			leftHandTargetWorldPosition = Effectors.LeftHandTarget.GlobalPosition,
			leftHandTargetWorldRotation = Quaternion.FromEuler(Effectors.LeftHandTarget.GlobalRotation),
			rightHandTargetWorldPosition = Effectors.RightHandTarget.GlobalPosition,
			rightHandTargetWorldRotation = Quaternion.FromEuler(Effectors.RightHandTarget.GlobalRotation),
			
			leftFootTargetWorldPosition = Effectors.LeftFootTarget.GlobalPosition,
			leftFootTargetWorldRotation = Quaternion.FromEuler(Effectors.LeftFootTarget.GlobalRotation),
			rightFootTargetWorldPosition = Effectors.RightFootTarget.GlobalPosition,
			rightFootTargetWorldRotation = Quaternion.FromEuler(Effectors.RightFootTarget.GlobalRotation),
			
			useChest = Effectors.UseChest,
			chestTargetWorldPosition = Effectors.ChestTarget.GlobalPosition,
			chestTargetWorldRotation = Quaternion.FromEuler(Effectors.ChestTarget.GlobalRotation),
			alsoUseChestToMoveNeck = Effectors.AlsoUseChestToMoveNeck,
			
			useLeftLowerArm = Effectors.UseLeftLowerArm,
			leftLowerArmWorldPosition = Effectors.LeftLowerArmTarget.GlobalPosition,
			leftLowerArmWorldRotation = Quaternion.FromEuler(Effectors.LeftLowerArmTarget.GlobalRotation),
			useRightLowerArm = Effectors.UseRightLowerArm,
			rightLowerArmWorldPosition = Effectors.RightLowerArmTarget.GlobalPosition,
			rightLowerArmWorldRotation = Quaternion.FromEuler(Effectors.RightLowerArmTarget.GlobalRotation),
			
			headAlignmentMattersMore = !Effectors.HipPositionMattersMore,
			allowContortionist = Effectors.Contortionist,
			doNotPreserveHipsToNeckCurvatureLimit = Effectors.DoNotPreserveHipsToNeckCurvatureLimit,
			
			useStraddlingLeftLeg = Effectors.UseStraddlingLeftLeg,
			useStraddlingRightLeg = Effectors.UseStraddlingRightLeg,
			groundedStraddlingLeftLegWorldPosition = Effectors.GroundedStraddlingLeftLeg.GlobalPosition,
			groundedStraddlingLeftLegWorldRotation = Quaternion.FromEuler(Effectors.GroundedStraddlingLeftLeg.GlobalRotation),
			groundedStraddlingRightLegWorldPosition = Effectors.GroundedStraddlingRightLeg.GlobalPosition,
			groundedStraddlingRightLegWorldRotation = Quaternion.FromEuler(Effectors.GroundedStraddlingRightLeg.GlobalRotation),
			
			solveSpine = _solveSpine,
			solveLeftLeg = _solveLeftLeg,
			solveRightLeg = _solveRightLeg,
			solveLeftArm = _solveLeftArm,
			solveRightArm = _solveRightArm,
			
			legStruggleStart = Effectors.LegStruggleStart,
			legStruggleEnd = Effectors.LegStruggleEnd,
			armStruggleStart = Effectors.ArmStruggleStart,
			armStruggleEnd = Effectors.ArmStruggleEnd,
			
			useShoulder = Effectors.UseShoulder,
			shoulderForwardAngleMultiplier = Effectors.ShoulderForwardAngleMultiplier,
			shoulderUpwardAngleMultiplier = Effectors.ShoulderUpwardAngleMultiplier,
			
			improveSpineBuckling = Effectors.ImproveSpineBuckling,
			
			providedLossyScale = _humanoid.ResolveLossyScale(),
			
			fabrikIterations = OverrideDefaultFabrikIterationCount ? FabrikIterations : HIKSpineSolver.Iterations,
			
			__useFakeDoubleJointedKnees = Effectors.UseFakeDoubleJointedKnees,
			
			selfParentLeftHandNullable = selfParentLeftHand,
			selfParentRightHandNullable = selfParentRightHand,
		}, _ikSnapshot, DebugDrawSolver);
	}

	public void ApplySnapshot()
	{
		_humanoid.SetAbsolutePosition(Hips, _ikSnapshot.absolutePos[(int)Hips]);
		_humanoid.SetAbsoluteRotation(Hips, ConvertSnapshotRotationToBoneRotation(_ikSnapshot, definition, Hips));
		if (_solveSpine) Apply(CopyOrderSpineAndShoulders);
		if (!_useFakeDoubleJointedKneesWasEverEnabled && Effectors.UseFakeDoubleJointedKnees <= 0f)
		{
			if (_solveLeftLeg) Apply(CopyOrderLeftLeg);
			if (_solveRightLeg) Apply(CopyOrderRightLeg);
		}
		else
		{
			_useFakeDoubleJointedKneesWasEverEnabled = true;
			if (_solveLeftLeg)
			{
				Apply(LeftUpperLeg);
				_humanoid.SetAbsolutePosition(LeftLowerLeg, _ikSnapshot.absolutePos[(int)LeftLowerLeg]);
				Apply(LeftLowerLeg, LeftFoot);
				_humanoid.SetAbsolutePosition(LeftFoot, _ikSnapshot.absolutePos[(int)LeftFoot]);
			};
			if (_solveRightLeg)
			{
				Apply(RightUpperLeg);
				_humanoid.SetAbsolutePosition(RightLowerLeg, _ikSnapshot.absolutePos[(int)RightLowerLeg]);
				Apply(RightLowerLeg, RightFoot);
				_humanoid.SetAbsolutePosition(RightFoot, _ikSnapshot.absolutePos[(int)RightFoot]);
			}
		}
		if (_solveLeftArm) Apply(CopyOrderLeftArm);
		if (_solveRightArm) Apply(CopyOrderRightArm);
	}

	private void Apply(params HIKBodyBones[] bones)
	{
		foreach (var boneId in bones)
		{
			var index = (int)boneId;
			if (definition.dataHasBone[index])
			{
				_humanoid.SetAbsoluteRotation(boneId, ConvertSnapshotRotationToBoneRotation(_ikSnapshot, definition, boneId));
			}
		}
	}

	private static Quaternion ConvertSnapshotRotationToBoneRotation(HIKSnapshot ikSnapshot, HIKAvatarDefinition definition, HIKBodyBones bone)
	{
		return hvr_godot_math.mul(ikSnapshot.absoluteRot[(int)bone], definition.dataInversePostRot[(int)bone]);
	}

	private void DrawArmChain(HIKBodyBones[] array)
	{
		Vector3 prevPos = _humanoid.GetWorldSpacePosition(array[0]) + Vector3.Up * 0.001f;
		for (var i = 1; i < array.Length; i++)
		{
			var boneId = array[i];
			var index = (int)boneId;
			if (definition.dataHasBone[index])
			{
				var newPos = _humanoid.GetWorldSpacePosition(boneId) + Vector3.Up * 0.001f;
				DebugDraw3D.DrawLine(prevPos, newPos);
				prevPos = newPos;
			}
		}
	}
}
#endif
