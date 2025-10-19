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
using System;
using System.Collections.Generic;
using Godot;
using HVR.IK.FullTiger;

namespace HVR.IK.Gd.FullTiger;

public class HIKGdHumanoidMapper
{
    private readonly Skeleton3D _skeleton;
    private readonly Dictionary<string, int> _boneNameToSkeletonIndex = new();
    private readonly int[] _boneToSkeletonIndexOrMinusOne = new int[Enum.GetValues<HIKBodyBones>().Length];

    public HIKGdHumanoidMapper(Skeleton3D skeleton)
    {
        _skeleton = skeleton;
        
        for (var boneIndex = 0; boneIndex < skeleton.GetBoneCount(); boneIndex++)
        {
            _boneNameToSkeletonIndex[skeleton.GetBoneName(boneIndex)] = boneIndex;
        }

        var bones = Enum.GetValues<HIKBodyBones>();
        foreach (var bone in bones)
        {
            var found = false;
            var namesToTry = NamesToTry(bone);
            foreach (var name in namesToTry)
            {
                if (_boneNameToSkeletonIndex.TryGetValue(name, out var boneIndex))
                {
                    found = true;
                    _boneToSkeletonIndexOrMinusOne[(int)bone] = boneIndex;
                    break;
                }
            }

            if (!found)
            {
                _boneToSkeletonIndexOrMinusOne[(int)bone] = -1;
            }
        }

        {
            var results = Enum.GetValues<HIKBodyBones>();
            foreach (var bone in results)
            {
                var skeletonIndex = _boneToSkeletonIndexOrMinusOne[(int)bone];
                if (skeletonIndex != -1)
                {
                    GD.Print($"Humanoid bone {bone} is named {skeleton.GetBoneName(skeletonIndex)} (at index {skeletonIndex})");
                }
                else
                {
                    GD.Print($"Humanoid bone {bone} was not found");
                }
            }
        }
    }

    private Quaternion Propose(HIKBodyBones bone)
    {
        var indeterminate = Quaternion.Identity;
        var straight = hvr_godot_helper_quaternion.LookRotationSafe(Vector3.Right, Vector3.Forward);
        var downwards = hvr_godot_helper_quaternion.LookRotationSafe(Vector3.Left, Vector3.Forward);
        var leftHandWards = hvr_godot_helper_quaternion.LookRotationSafe(Vector3.Forward, Vector3.Up);
        var rightHandWards = hvr_godot_helper_quaternion.LookRotationSafe(Vector3.Forward, Vector3.Down);
        // var leftHandWards = hvr_godot_helper_quaternion.LookRotationSafe(Vector3.Forward, Vector3.Down);
        // var rightHandWards = hvr_godot_helper_quaternion.LookRotationSafe(Vector3.Forward, Vector3.Up);
        return bone switch
        {
            HIKBodyBones.Hips => straight,
            HIKBodyBones.LeftUpperLeg => downwards,
            HIKBodyBones.RightUpperLeg => downwards,
            HIKBodyBones.LeftLowerLeg => downwards,
            HIKBodyBones.RightLowerLeg => downwards,
            HIKBodyBones.LeftFoot => downwards,
            HIKBodyBones.RightFoot => downwards,
            HIKBodyBones.Spine => straight,
            HIKBodyBones.Chest => straight,
            HIKBodyBones.Neck => straight,
            HIKBodyBones.Head => straight,
            HIKBodyBones.LeftShoulder => leftHandWards,
            HIKBodyBones.RightShoulder => rightHandWards,
            HIKBodyBones.LeftUpperArm => leftHandWards,
            HIKBodyBones.RightUpperArm => rightHandWards,
            HIKBodyBones.LeftLowerArm => leftHandWards,
            HIKBodyBones.RightLowerArm => rightHandWards,
            HIKBodyBones.LeftHand => leftHandWards,
            HIKBodyBones.RightHand => rightHandWards,
            HIKBodyBones.LeftToes => downwards,
            HIKBodyBones.RightToes => downwards,
            HIKBodyBones.LeftEye => straight,
            HIKBodyBones.RightEye => straight,
            HIKBodyBones.Jaw => straight,
            HIKBodyBones.LeftThumbProximal => indeterminate,
            HIKBodyBones.LeftThumbIntermediate => indeterminate,
            HIKBodyBones.LeftThumbDistal => indeterminate,
            HIKBodyBones.LeftIndexProximal => indeterminate,
            HIKBodyBones.LeftIndexIntermediate => indeterminate,
            HIKBodyBones.LeftIndexDistal => indeterminate,
            HIKBodyBones.LeftMiddleProximal => indeterminate,
            HIKBodyBones.LeftMiddleIntermediate => indeterminate,
            HIKBodyBones.LeftMiddleDistal => indeterminate,
            HIKBodyBones.LeftRingProximal => indeterminate,
            HIKBodyBones.LeftRingIntermediate => indeterminate,
            HIKBodyBones.LeftRingDistal => indeterminate,
            HIKBodyBones.LeftLittleProximal => indeterminate,
            HIKBodyBones.LeftLittleIntermediate => indeterminate,
            HIKBodyBones.LeftLittleDistal => indeterminate,
            HIKBodyBones.RightThumbProximal => indeterminate,
            HIKBodyBones.RightThumbIntermediate => indeterminate,
            HIKBodyBones.RightThumbDistal => indeterminate,
            HIKBodyBones.RightIndexProximal => indeterminate,
            HIKBodyBones.RightIndexIntermediate => indeterminate,
            HIKBodyBones.RightIndexDistal => indeterminate,
            HIKBodyBones.RightMiddleProximal => indeterminate,
            HIKBodyBones.RightMiddleIntermediate => indeterminate,
            HIKBodyBones.RightMiddleDistal => indeterminate,
            HIKBodyBones.RightRingProximal => indeterminate,
            HIKBodyBones.RightRingIntermediate => indeterminate,
            HIKBodyBones.RightRingDistal => indeterminate,
            HIKBodyBones.RightLittleProximal => indeterminate,
            HIKBodyBones.RightLittleIntermediate => indeterminate,
            HIKBodyBones.RightLittleDistal => indeterminate,
            HIKBodyBones.UpperChest => straight,
            HIKBodyBones.LastBone => indeterminate,
            _ => throw new ArgumentOutOfRangeException(nameof(bone), bone, null)
        };
    }

    public bool HasBone(HIKBodyBones bone)
    {
        return _boneToSkeletonIndexOrMinusOne[(int)bone] != -1;
    }
    
    public int GetBoneIndexOrMinusOne(HIKBodyBones bone)
    {
        return _boneToSkeletonIndexOrMinusOne[(int)bone];
    }

    public Transform3D GetBoneTransformAtRest(HIKBodyBones bone)
    {
        return _skeleton.GetBoneRest(_boneToSkeletonIndexOrMinusOne[(int)bone]);
    }

    public Vector3 GetLocalPosition(HIKBodyBones bone)
    {
        return _skeleton.GetBonePose(_boneToSkeletonIndexOrMinusOne[(int)bone]).Origin;
    }

    public Quaternion GetLocalRotation(HIKBodyBones bone)
    {
        return _skeleton.GetBonePose(_boneToSkeletonIndexOrMinusOne[(int)bone]).Basis.GetRotationQuaternion();
    }

    public Vector3 GetLocalScale(HIKBodyBones bone)
    {
        return _skeleton.GetBonePose(_boneToSkeletonIndexOrMinusOne[(int)bone]).Basis.Scale;
    }

    public Quaternion GetPostRotation(HIKBodyBones bone)
    {
        return MomentarilySuggestEquivalentPostRotationVectorUsingCurrentPose(bone);
    }

    private Quaternion MomentarilySuggestEquivalentPostRotationVectorUsingCurrentPose(HIKBodyBones bone)
    {
        var boneWorldSpaceRotation = GetWorldSpaceRotation(bone);
        var proposeAlignedForBone = Propose(bone);
        return boneWorldSpaceRotation.Inverse() * proposeAlignedForBone;
    }

    public Vector3 ResolveLossyScale()
    {
        return _skeleton.GlobalTransform.Basis.Scale;
    }

    public Quaternion GetWorldSpaceHipsRotation()
    {
        return _skeleton.GlobalTransform.Basis.GetRotationQuaternion()
            * _skeleton.GetBoneGlobalPose(_boneToSkeletonIndexOrMinusOne[(int)HIKBodyBones.Hips]).Basis.GetRotationQuaternion();
    }

    public Quaternion GetWorldSpaceRotation(HIKBodyBones bone)
    {
        return _skeleton.GlobalTransform.Basis.GetRotationQuaternion()
               * _skeleton.GetBoneGlobalPose(_boneToSkeletonIndexOrMinusOne[(int)bone]).Basis.GetRotationQuaternion();
    }

    public Vector3 GetWorldSpacePosition(HIKBodyBones bone)
    {
        return _skeleton.GlobalTransform * _skeleton.GetBoneGlobalPose(_boneToSkeletonIndexOrMinusOne[(int)bone]).Origin;
    }

    public void SetAbsolutePosition(HIKBodyBones bone, Vector3 absolutePos)
    {
        var parentSkeletonIndex = _skeleton.GetBoneParent(_boneToSkeletonIndexOrMinusOne[(int)bone]);
        if (parentSkeletonIndex != -1)
        {
            var parentGlobalTransform = _skeleton.GlobalTransform * _skeleton.GetBoneGlobalPose(parentSkeletonIndex);
            var relativePos = parentGlobalTransform.Inverse() * absolutePos;
            _skeleton.SetBonePosePosition(_boneToSkeletonIndexOrMinusOne[(int)bone], relativePos);
        }
        else
        {
            _skeleton.SetBonePosePosition(_boneToSkeletonIndexOrMinusOne[(int)bone], _skeleton.GlobalTransform.Inverse() * absolutePos);
        }
    }

    public void SetAbsoluteRotation(HIKBodyBones bone, Quaternion absoluteRot)
    {
        var parentSkeletonIndex = _skeleton.GetBoneParent(_boneToSkeletonIndexOrMinusOne[(int)bone]);
        if (parentSkeletonIndex != -1)
        {
            var worldSpaceRotationOfParent = _skeleton.GlobalTransform.Basis.GetRotationQuaternion()
                                     * _skeleton.GetBoneGlobalPose(parentSkeletonIndex).Basis.GetRotationQuaternion();

            _skeleton.SetBonePoseRotation(_boneToSkeletonIndexOrMinusOne[(int)bone], worldSpaceRotationOfParent.Inverse() * absoluteRot);
        }
        else
        {
            _skeleton.SetBonePoseRotation(_boneToSkeletonIndexOrMinusOne[(int)bone], _skeleton.GlobalTransform.Basis.GetRotationQuaternion().Inverse() * absoluteRot);
        }
    }

    private static string[] NamesToTry(HIKBodyBones bone)
    {
        return bone switch
        {
            HIKBodyBones.Hips => ["Hips"],
            HIKBodyBones.LeftUpperLeg => ["LeftUpperLeg", "LeftUpLeg"],
            HIKBodyBones.RightUpperLeg => ["RightUpperLeg", "RightUpLeg"],
            HIKBodyBones.LeftLowerLeg => ["LeftLowerLeg", "LeftLeg"],
            HIKBodyBones.RightLowerLeg => ["RightLowerLeg", "RightLeg"],
            HIKBodyBones.LeftFoot => ["LeftFoot"],
            HIKBodyBones.RightFoot => ["RightFoot"],
            HIKBodyBones.Spine => ["Spine"],
            HIKBodyBones.Chest => ["Chest"],
            HIKBodyBones.Neck => ["Neck"],
            HIKBodyBones.Head => ["Head"],
            HIKBodyBones.LeftShoulder => ["LeftShoulder"],
            HIKBodyBones.RightShoulder => ["RightShoulder"],
            HIKBodyBones.LeftUpperArm => ["LeftUpperArm", "LeftArm"],
            HIKBodyBones.RightUpperArm => ["RightUpperArm", "RightArm"],
            HIKBodyBones.LeftLowerArm => ["LeftLowerArm", "LeftForeArm"],
            HIKBodyBones.RightLowerArm => ["RightLowerArm", "RightForeArm"],
            HIKBodyBones.LeftHand => ["LeftHand"],
            HIKBodyBones.RightHand => ["RightHand"],
            HIKBodyBones.LeftToes => ["LeftToes", "LeftToeBase"],
            HIKBodyBones.RightToes => ["RightToes", "RightToeBase"],
            HIKBodyBones.LeftEye => ["LeftEye"],
            HIKBodyBones.RightEye => ["RightEye"],
            HIKBodyBones.Jaw => ["Jaw"],
            HIKBodyBones.LeftThumbProximal => ["LeftThumbProximal", "LeftHandThumb1"],
            HIKBodyBones.LeftThumbIntermediate => ["LeftThumbIntermediate", "LeftHandThumb2"],
            HIKBodyBones.LeftThumbDistal => ["LeftThumbDistal", "LeftHandThumb3"],
            HIKBodyBones.LeftIndexProximal => ["LeftIndexProximal", "LeftHandIndex1"],
            HIKBodyBones.LeftIndexIntermediate => ["LeftIndexIntermediate", "LeftHandIndex2"],
            HIKBodyBones.LeftIndexDistal => ["LeftIndexDistal", "LeftHandIndex3"],
            HIKBodyBones.LeftMiddleProximal => ["LeftMiddleProximal", "LeftHandMiddle1"],
            HIKBodyBones.LeftMiddleIntermediate => ["LeftMiddleIntermediate", "LeftHandMiddle2"],
            HIKBodyBones.LeftMiddleDistal => ["LeftMiddleDistal", "LeftHandMiddle3"],
            HIKBodyBones.LeftRingProximal => ["LeftRingProximal", "LeftHandRing1"],
            HIKBodyBones.LeftRingIntermediate => ["LeftRingIntermediate", "LeftHandRing2"],
            HIKBodyBones.LeftRingDistal => ["LeftRingDistal", "LeftHandRing3"],
            HIKBodyBones.LeftLittleProximal => ["LeftLittleProximal", "LeftHandPinky1"],
            HIKBodyBones.LeftLittleIntermediate => ["LeftLittleIntermediate", "LeftHandPinky2"],
            HIKBodyBones.LeftLittleDistal => ["LeftLittleDistal", "LeftHandPinky3"],
            HIKBodyBones.RightThumbProximal => ["RightThumbProximal", "RightHandThumb1"],
            HIKBodyBones.RightThumbIntermediate => ["RightThumbIntermediate", "RightHandThumb2"],
            HIKBodyBones.RightThumbDistal => ["RightThumbDistal", "RightHandThumb3"],
            HIKBodyBones.RightIndexProximal => ["RightIndexProximal", "RightHandIndex1"],
            HIKBodyBones.RightIndexIntermediate => ["RightIndexIntermediate", "RightHandIndex2"],
            HIKBodyBones.RightIndexDistal => ["RightIndexDistal", "RightHandIndex3"],
            HIKBodyBones.RightMiddleProximal => ["RightMiddleProximal", "RightHandMiddle1"],
            HIKBodyBones.RightMiddleIntermediate => ["RightMiddleIntermediate", "RightHandMiddle2"],
            HIKBodyBones.RightMiddleDistal => ["RightMiddleDistal", "RightHandMiddle3"],
            HIKBodyBones.RightRingProximal => ["RightRingProximal", "RightHandRing1"],
            HIKBodyBones.RightRingIntermediate => ["RightRingIntermediate", "RightHandRing2"],
            HIKBodyBones.RightRingDistal => ["RightRingDistal", "RightHandRing3"],
            HIKBodyBones.RightLittleProximal => ["RightLittleProximal", "RightHandPinky1"],
            HIKBodyBones.RightLittleIntermediate => ["RightLittleIntermediate", "RightHandPinky2"],
            HIKBodyBones.RightLittleDistal => ["RightLittleDistal", "RightHandPinky3"],
            HIKBodyBones.UpperChest => ["UpperChest"],
            HIKBodyBones.LastBone => ["LastBone"],
            _ => throw new ArgumentOutOfRangeException()
        };
    }
}
#endif