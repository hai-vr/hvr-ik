#if !UNITY_2022_1_OR_NEWER //__GODOT
using Godot;
using System;

public partial class HIKGdEffectors : Node
{
	[Export] public string Message { get; set; } = "This is printed every frame!";
	[Export] public Skeleton3D Skeleton { get; set; }
	[Export] public Node3D SkeletonSearch { get; set; }
	
	public override void _Ready()
	{
		if (SkeletonSearch != null && Skeleton == null)
		{
			Skeleton = FindFirstNodeWithScript<Skeleton3D>(SkeletonSearch);
		}

		if (Skeleton == null) return;
	}

	public override void _Process(double delta)
	{
		if (Skeleton == null) return;
		
		GD.Print(Message);
	}
	
	private T FindFirstNodeWithScript<T>(Node node) where T : class
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