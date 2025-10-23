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

#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
using UnityEngine;

namespace HVR.IK.FullTiger
{
    public class HIKFullTigerNoUpdates : MonoBehaviour
    {
        public Animator animator;
        public HIKEffectors effectors;
        public HIKEnvironmental environmental;
        
        private HIKAvatarDefinition definition = new();
        private HIKSolver _ikSolver;
        
        private readonly Transform[] _bones = new Transform[(int)HumanBodyBones.LastBone];
        
        private void Awake()
        {
            definition = HIKFullTiger.SolveDefinition(animator, definition, _bones);
            
            // Order matters: This must be instantiated AFTER definition is initialized
            _ikSolver = new HIKSolver(definition, new HIKLookupTables(HIKFullTiger.ParseLookup()));
        }

    }
}
#endif