using System;

namespace HVR.IK.FullTiger
{
    internal class/*was_struct*/ HIKSpineData<T>
    {
        // CURSED: We're actively avoiding the use of NativeArray (and arrays) here because it's causing an absolute pain of issues with Burst and Hot Reload.
        // Maybe there's a better way to do this.
        internal T eltA;
        internal T eltB;
        internal T eltC;
        internal T eltD;
        internal T eltE;
        internal int Length;

        public HIKSpineData(int length)
        {
            Length = length;
            eltA = default;
            eltB = default;
            eltC = default;
            eltD = default;
            eltE = default;
        }

        public T this[int index]
        {
            get
            {
                return index switch
                {
                    0 => eltA,
                    1 => eltB,
                    2 => eltC,
                    3 => eltD,
                    4 => eltE,
                    _ => throw new IndexOutOfRangeException($"Not accepted {index}")
                };
            }
            set
            {
                _ = index switch
                {
                    0 => eltA = value,
                    1 => eltB = value,
                    2 => eltC = value,
                    3 => eltD = value,
                    4 => eltE = value,
                    _ => throw new IndexOutOfRangeException($"Not accepted {index}")
                };
            }
        }
    }
    
    internal struct/*reconverted_to_struct*/ HIKBoneData<T>
    {
        // CURSED: We're actively avoiding the use of NativeArray (and arrays) here because it's causing an absolute pain of issues with Burst and Hot Reload.
        // Maybe there's a better way to do this.
        internal T eltHips; internal T eltSpine; internal T eltChest; internal T eltUpperChest; internal T eltNeck; internal T eltHead;
        internal T eltLeftShoulder; internal T eltLeftUpperArm; internal T eltLeftLowerArm; internal T eltLeftHand;
        internal T eltRightShoulder; internal T eltRightUpperArm; internal T eltRightLowerArm; internal T eltRightHand;
        internal T eltLeftUpperLeg; internal T eltLeftLowerLeg; internal T eltLeftFoot; internal T eltLeftToes;
        internal T eltRightUpperLeg; internal T eltRightLowerLeg; internal T eltRightFoot; internal T eltRightToes;

        public T this[int boneIndex]
        {
            get
            {
                var bone = (HIKBodyBones)boneIndex;
                return bone switch
                {
                    HIKBodyBones.Hips => eltHips,
                    HIKBodyBones.Spine => eltSpine,
                    HIKBodyBones.Chest => eltChest,
                    HIKBodyBones.UpperChest => eltUpperChest,
                    HIKBodyBones.Neck => eltNeck,
                    HIKBodyBones.Head => eltHead,
                    HIKBodyBones.LeftShoulder => eltLeftShoulder,
                    HIKBodyBones.LeftUpperArm => eltLeftUpperArm,
                    HIKBodyBones.LeftLowerArm => eltLeftLowerArm,
                    HIKBodyBones.LeftHand => eltLeftHand,
                    HIKBodyBones.RightShoulder => eltRightShoulder,
                    HIKBodyBones.RightUpperArm => eltRightUpperArm,
                    HIKBodyBones.RightLowerArm => eltRightLowerArm,
                    HIKBodyBones.RightHand => eltRightHand,
                    HIKBodyBones.LeftUpperLeg => eltLeftUpperLeg,
                    HIKBodyBones.LeftLowerLeg => eltLeftLowerLeg,
                    HIKBodyBones.LeftFoot => eltLeftFoot,
                    HIKBodyBones.LeftToes => eltLeftToes,
                    HIKBodyBones.RightUpperLeg => eltRightUpperLeg,
                    HIKBodyBones.RightLowerLeg => eltRightLowerLeg,
                    HIKBodyBones.RightFoot => eltRightFoot,
                    HIKBodyBones.RightToes => eltRightToes,
                    _ => throw new IndexOutOfRangeException($"Not accepted bone {boneIndex}")
                };
            }
            set
            {
                var bone = (HIKBodyBones)boneIndex;
                _ = bone switch
                {
                    HIKBodyBones.Hips => eltHips = value,
                    HIKBodyBones.Spine => eltSpine = value,
                    HIKBodyBones.Chest => eltChest = value,
                    HIKBodyBones.UpperChest => eltUpperChest = value,
                    HIKBodyBones.Neck => eltNeck = value,
                    HIKBodyBones.Head => eltHead = value,
                    HIKBodyBones.LeftShoulder => eltLeftShoulder = value,
                    HIKBodyBones.LeftUpperArm => eltLeftUpperArm = value,
                    HIKBodyBones.LeftLowerArm => eltLeftLowerArm = value,
                    HIKBodyBones.LeftHand => eltLeftHand = value,
                    HIKBodyBones.RightShoulder => eltRightShoulder = value,
                    HIKBodyBones.RightUpperArm => eltRightUpperArm = value,
                    HIKBodyBones.RightLowerArm => eltRightLowerArm = value,
                    HIKBodyBones.RightHand => eltRightHand = value,
                    HIKBodyBones.LeftUpperLeg => eltLeftUpperLeg = value,
                    HIKBodyBones.LeftLowerLeg => eltLeftLowerLeg = value,
                    HIKBodyBones.LeftFoot => eltLeftFoot = value,
                    HIKBodyBones.LeftToes => eltLeftToes = value,
                    HIKBodyBones.RightUpperLeg => eltRightUpperLeg = value,
                    HIKBodyBones.RightLowerLeg => eltRightLowerLeg = value,
                    HIKBodyBones.RightFoot => eltRightFoot = value,
                    HIKBodyBones.RightToes => eltRightToes = value,
                    _ => throw new IndexOutOfRangeException($"Not accepted bone {boneIndex}")
                };
            }
        }
    }
}