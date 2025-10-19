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
using System;
using System.Linq;
using HVR.IK.FullTiger;

namespace HVR.IK.Gd.FullTiger;

/// <summary>
/// Creates the end effectors that will be used by the IK solver.
/// The end effectors placements should be modified by external modules before the IK solver runs.
/// </summary>
public partial class HIKGdEffectors : Node
{
	[Export] public Skeleton3D Skeleton { get; set; }
	[Export] public Node3D SkeletonSearch { get; set; }

	[ExportGroup("Classic effectors")]
	[Export] public Node3D RuntimeTargets { get; set; }
	[Export] public Node3D HipTarget { get; set; }
	[Export] public Node3D HeadTarget { get; set; }
	[Export] public Node3D LeftHandTarget { get; set; }
	[Export] public Node3D RightHandTarget { get; set; }
	[Export] public Node3D LeftFootTarget { get; set; }
	[Export] public Node3D RightFootTarget { get; set; }
		
	[ExportGroup("Spine")]
	[Export] public bool HipPositionMattersMore { get; set; }
	[Export] public bool Contortionist { get; set; }
	[Export] public bool DoNotPreserveHipsToNeckCurvatureLimit { get; set; }
	[Export /*RANGE(0f, 1f)*/] public float ImproveSpineBuckling { get; set; } = 1f;

	[ExportGroup("Chest effector")]
	[Export /*RANGE(0f, 1f)*/] public float UseChest { get; set; }
	[Export] public Node3D ChestTarget { get; set; }
	[Export /*RANGE(0f, 1f)*/] public float AlsoUseChestToMoveNeck { get; set; }
		
	[ExportGroup("Arm bend")]
	[Export /*RANGE(0f, 1f)*/] public float UseLeftLowerArm { get; set; }
	[Export] public Node3D LeftLowerArmTarget { get; set; }
	[Export /*RANGE(0f, 1f)*/] public float UseRightLowerArm { get; set; }
	[Export] public Node3D RightLowerArmTarget { get; set; }
		
	[ExportGroup("Struggle")]
	[Export] public float LegStruggleStart { get; set; } = HIKObjective.StruggleStart;
	[Export] public float LegStruggleEnd { get; set; } = HIKObjective.StruggleEnd;
	[Export] public float ArmStruggleStart { get; set; } = HIKObjective.StruggleStart;
	[Export] public float ArmStruggleEnd { get; set; } = HIKObjective.StruggleEnd;
		
	[ExportGroup("Shoulder")]
	[Export /*RANGE(0f, 1f)*/] public float UseShoulder { get; set; } = 0f;
	[Export /*RANGE(0f, 1f)*/] public float ShoulderForwardAngleMultiplier { get; set; } = 1f;
	[Export /*RANGE(0f, 1f)*/] public float ShoulderUpwardAngleMultiplier { get; set; } = 1f;
		
	[ExportGroup("Straddling")]
	[Export] public bool UseStraddlingLeftLeg { get; set; }
	[Export] public Node3D GroundedStraddlingLeftLeg { get; set; }
	[Export] public bool UseStraddlingRightLeg { get; set; }
	[Export] public Node3D GroundedStraddlingRightLeg { get; set; }
		
	[ExportGroup("Self-parenting, Left Hand")]
	[Export /*RANGE(0f, 1f)*/] public float UseSelfParentLeftHand { get; set; }
	[Export] public HIKBodyBones SelfParentLeftHandBone { get; set; }
	[Export] public Vector3 SelfParentLeftHandRelativePosition { get; set; }
	[Export] public Vector3 SelfParentLeftHandRelativeRotationEuler { get; set; }
		
	[ExportGroup("Self-parenting, Right Hand")]
	[Export /*RANGE(0f, 1f)*/] public float UseSelfParentRightHand { get; set; }
	[Export] public HIKBodyBones SelfParentRightHandBone { get; set; }
	[Export] public Vector3 SelfParentRightHandRelativePosition { get; set; }
	[Export] public Vector3 SelfParentRightHandRelativeRotationEuler { get; set; }

	[ExportGroup("Experimental (CHANGES POSITION)")]
	[Export /*RANGE(0f, 1f)*/] public float UseFakeDoubleJointedKnees { get; set; } = 0f;

	private HIKGdHumanoidMapper _humanoid;
	private Vector3[] _tPosePos;
	private Quaternion[] _tPoseRot;

	public bool IsInitialized() => _isInitialized;
	private bool _isInitialized;

	public override void _Ready()
	{
		if (SkeletonSearch != null && Skeleton == null)
		{
			Skeleton = HIKGdUtil.FindFirstNodeWithScript<Skeleton3D>(SkeletonSearch);
		}

		if (Skeleton == null)
		{
			GD.PrintErr("HIKGdEffectors: No Skeleton3D found");
			return;
		}
		
		for (var boneIndex = 0; boneIndex < Skeleton.GetBoneCount(); boneIndex++)
		{
			GD.Print($"{boneIndex} is {Skeleton.GetBoneName(boneIndex)}");
		}

		_humanoid = new HIKGdHumanoidMapper(Skeleton);

		InitializeTargets();
	}

	private void InitializeTargets()
	{
		if (RuntimeTargets == null) 
			RuntimeTargets = CreateNode3D("RuntimeTargets", this);

		if (HipTarget == null) 
			HipTarget = CreateTarget(HIKBodyBones.Hips, _humanoid.GetBoneIndexOrMinusOne(HIKBodyBones.Hips), "HipTarget");
		if (HeadTarget == null) 
			HeadTarget = CreateTarget(HIKBodyBones.Head, _humanoid.GetBoneIndexOrMinusOne(HIKBodyBones.Head), "HeadTarget");
		if (LeftHandTarget == null) 
			LeftHandTarget = CreateTarget(HIKBodyBones.LeftHand, _humanoid.GetBoneIndexOrMinusOne(HIKBodyBones.LeftHand), "LeftHandTarget");
		if (RightHandTarget == null) 
			RightHandTarget = CreateTarget(HIKBodyBones.RightHand, _humanoid.GetBoneIndexOrMinusOne(HIKBodyBones.RightHand), "RightHandTarget");
		if (LeftFootTarget == null) 
			LeftFootTarget = CreateTarget(HIKBodyBones.LeftFoot, _humanoid.GetBoneIndexOrMinusOne(HIKBodyBones.LeftFoot), "LeftFootTarget");
		if (RightFootTarget == null) 
			RightFootTarget = CreateTarget(HIKBodyBones.RightFoot, _humanoid.GetBoneIndexOrMinusOne(HIKBodyBones.RightFoot), "RightFootTarget");
		if (GroundedStraddlingLeftLeg == null) 
			GroundedStraddlingLeftLeg = CreateTarget(HIKBodyBones.LeftLowerLeg, _humanoid.GetBoneIndexOrMinusOne(HIKBodyBones.LeftLowerLeg), "GroundedStraddlingLeftLeg");
		if (GroundedStraddlingRightLeg == null) 
			GroundedStraddlingRightLeg = CreateTarget(HIKBodyBones.RightLowerLeg, _humanoid.GetBoneIndexOrMinusOne(HIKBodyBones.RightLowerLeg), "GroundedStraddlingRightLeg");
			
		if (ChestTarget == null) 
			ChestTarget = CreateTarget(HIKBodyBones.Chest, _humanoid.GetBoneIndexOrMinusOne(HIKBodyBones.Chest), "ChestTarget");
		if (LeftLowerArmTarget == null) 
			LeftLowerArmTarget = CreateTarget(HIKBodyBones.LeftLowerArm, _humanoid.GetBoneIndexOrMinusOne(HIKBodyBones.LeftLowerArm), "LeftLowerArmTarget");
		if (RightLowerArmTarget == null) 
			RightLowerArmTarget = CreateTarget(HIKBodyBones.RightLowerArm, _humanoid.GetBoneIndexOrMinusOne(HIKBodyBones.RightLowerArm), "RightLowerArmTarget");

		var allTargets = AllTargetsStartingWithHead();
		_tPosePos = allTargets.Select(t => t.GlobalPosition).ToArray();
		_tPoseRot = allTargets.Select(t => Quaternion.FromEuler(t.GlobalRotation)).ToArray();
			
		_isInitialized = true;
	}

	public override void _ExitTree()
	{
		HipTarget = null;
		HeadTarget = null;
		LeftHandTarget = null;
		RightHandTarget = null;
		LeftFootTarget = null;
		RightFootTarget = null;
		GroundedStraddlingLeftLeg = null;
		GroundedStraddlingRightLeg = null;

		if (RuntimeTargets != null)
		{
			RuntimeTargets.QueueFree();
			RuntimeTargets = null;
		}
	}

	public void ApplyTPoseAndRepositionHeadToMatch(Vector3 pos, Quaternion rot)
	{
		ApplyTPose();
		var transferRotation = rot * HeadTarget.GlobalRotation.Inverse();
		var others = AllTargetsStartingWithHead().Skip(1).ToArray();
		var repositions = others.Select(t => HeadTarget.ToLocal(t.GlobalPosition)).ToArray();
			
		HeadTarget.GlobalPosition = pos;
		HeadTarget.GlobalRotation = rot.GetEuler();
			
		for (var index = 0; index < others.Length; index++)
		{
			var t = others[index];
			t.GlobalPosition = HeadTarget.ToGlobal(repositions[index]);
			t.GlobalRotation = transferRotation * t.GlobalRotation;
		}
	}

	public void ApplyTPose()
	{
		var targets = AllTargetsStartingWithHead();
		for (var index = 0; index < targets.Length; index++)
		{
			var target = targets[index];
			target.GlobalPosition = _tPosePos[index];
			target.GlobalRotation = _tPoseRot[index].GetEuler();
		}
	}

	private Node3D[] AllTargetsStartingWithHead()
	{
		return [HeadTarget, HipTarget, LeftHandTarget, RightHandTarget, LeftFootTarget, RightFootTarget];
	}

	private Node3D CreateTarget(HIKBodyBones bone, int boneIndex, string targetName)
	{
		if (boneIndex == -1)
		{
			GD.PrintErr($"HIKGdEffectors: Bone not found for target {targetName}");
			return CreateNode3D(targetName, RuntimeTargets);
		}

		var globalPos = _humanoid.GetWorldSpacePosition(bone);
		var globalRot = _humanoid.GetWorldSpaceRotation(bone) * _humanoid.GetPostRotation(bone);
			
		return CreateNode3D(targetName, RuntimeTargets, globalPos, globalRot);
	}

	private Node3D CreateNode3D(string nodeName, Node parent, Vector3? position = null, Quaternion? rotation = null)
	{
		var node = new Node3D { Name = nodeName };
		parent.AddChild(node);
			
		if (position.HasValue)
			node.GlobalPosition = position.Value;
		if (rotation.HasValue)
			node.GlobalRotation = rotation.Value.GetEuler();
				
		return node;
	}
}
#endif
