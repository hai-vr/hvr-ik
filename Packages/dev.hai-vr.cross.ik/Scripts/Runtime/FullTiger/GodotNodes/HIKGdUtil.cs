#if !UNITY_2022_1_OR_NEWER //__GODOT
using Godot;

namespace HVR.IK.Gd.FullTiger;

public class HIKGdUtil
{
    public static T FindFirstNodeWithScript<T>(Node node) where T : class
    {
        if (node.GetScript().AsGodotObject() is T script)
        {
            return script;
        }
	
        if (node is T directCast)
        {
            return directCast;
        }
	
        foreach (var child in node.GetChildren())
        {
            var result = FindFirstNodeWithScript<T>(child);
            if (result != null)
            {
                return result;
            }
        }
	
        return null;
    }
}
#endif