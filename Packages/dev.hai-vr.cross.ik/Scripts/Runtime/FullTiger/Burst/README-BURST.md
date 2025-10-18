Burst?
===

You might notice that this folder named "Burst" doesn't actually have anything Burst-compatible in it.

Here's what happened:

While adding the HIKFullTigerAnimationRigging component, I thought I'd also try to make the core solver Burst-compatible.
However, attempting to do so has caused a massive amount of issues with the [Hot Reload](https://hotreload.net/) plugin,
which I need to keep short iteration cycles while working on heuristics.

So, I've disabled Burst support for now.

The issues were the following:
- Inability to keep NativeArray consistent across reloads,
- Hard crashes when hot reloading,

I did try to hook into Hot Reload callbacks but the results were inconclusive.

I've also had to turn structs back into classes (notice the `class/*was_struct*/` in most of the files here),
and turn `= default;` into `= new();`.