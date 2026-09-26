# StructuralEngine

How a stick model is read, before any tool has an opinion about it. The shared
structural base of OtterLogic, and the structural counterpart of
[MachineLearning](https://github.com/Otter-Logic/MachineLearning): what every
structural toolkit needs, held once.

```
Core   Graphs                                  foundations, reference nothing
    \  /
  MachineLearning                              features, PCA, datasets
        |
  Unsupervised   Supervised  ...               the paradigms
        |
  StructuralEngine                             this repo: reading a model
        |
  StructuralDesign  Construction  Fabrication  BIM     the opinions
        |
     Rhino3D
```

It sits **on** the learning stack rather than beside it, because reading a model
already needs two of its mechanisms: the turn at which a run stops being one
member is learned from the model's own turns by `ValueBands`, and an assembly's
span and depth are its principal components. Copying either here would be the
duplication the layering exists to prevent.

## What is here

Lines as start and end points, surfaces as boundary corners, supports as points
in — the same arrays every analysis package and every Grasshopper wire can hand
over — and the structure they describe out, stage by stage:

| Stage | What is read | |
|---|---|---|
| Input | starts and ends of equal count, rings of at least three corners, finite coordinates; refused with a reason, never repaired | `ModelInput` |
| Joints | points within the join distance weld into one joint; a joint resting along an element joins it; elements become a graph joined where they meet, joints a graph joined along elements | `StructureGraph` |
| Geometry | each element's centroid, size, extent and aspect | `ElementGeometry` |
| Members | lines that carry straight on through a joint are one physical member, straightest pair first; how much of a turn still counts is learned from the model, between 5 and 30 degrees | `PhysicalMembers` |
| Assemblies | triangles sharing a side in one plane are one body — a truss, a braced bay; a member in two planes goes to the more upright; each body gets its own span and depth | `Assemblies` |
| Load paths | every element's weight drained to the supports as a potential flow, bending a hundred times softer than axial; hand-overs read joint by joint, summed between assemblies into a hand-over graph, loops folded, levels counted up from the ground with anything standing on the supports itself at level 0 | `LoadPaths` |
| Load tree | the path an engineer would draw, taken from the flow so the two agree: each joint hands to the neighbour it sends most to; what depends on each joint is its tributary, and a member's is the most any part of it carries; a stretch is on the path or it braces; the least resistance from a support over the flow's own conductances, so a storey up a column costs its length and the same reach along a beam a hundred times more; a joint held up through one joint alone is cantilevered | `LoadTree` |
| Pieces | each run cut into the pieces it is made in, where it hands a substantial share of its weight on part of the way along — to the ground, to another body, or to a member carrying on through the joint; each piece's span, unbraced length, uprightness and flow. Reads a frame as simply connected | `Pieces`, `LoadPaths.JointHandOvers` |
| Regions | where the member graph nearly comes apart: the Fiedler vector's sign is the side of the weakest cut and its size the depth into it, the eigenvalue with it how near the cut is to a split, and what each member alone strands | `Regions` |
| Orientation | how each line stands — level, pitched, plumb — from the model's own spread of inclinations, named by the nearest prototype rather than a cut-off | `LineOrientations` |

Three things are taken as true of every structure, and nothing else: gravity
points down, weight ends at the supports, and a line that carries straight on
through a joint is one member. There is no rule here that knows a column from a
beam or a truss from a shell, and nothing refers to x, y or position in plan —
turned about the vertical, a model reads the same. That is tested.

## What is deliberately not here

- **Opinions.** Which groups a structure falls into, what a piece is called, the
  order pieces go up in, how many kinds of connection a job has: each is a
  toolkit's claim, and lives in StructuralDesign, Construction or Fabrication.
  A test for whether something belongs here: would two toolkits reach the same
  answer from it? If one of them would want it different, it is theirs.
- **Grasshopper components.** This repo ships none. Its output is arrays, and a
  user meets it through the toolkits' tools.
- **Rhino types.** Plain coordinate arrays throughout, so the tests run anywhere.

## Rules

- Depends on [Unsupervised](https://github.com/Otter-Logic/Unsupervised), which
  carries [MachineLearning](https://github.com/Otter-Logic/MachineLearning),
  [Graphs](https://github.com/Otter-Logic/Graphs) and
  [Core](https://github.com/Otter-Logic/Core) transitively. Never a toolkit,
  never an adaptor.
- Two toolkits needing the same reading is the signal to move it here. One
  toolkit wanting it different is the signal that it was never a reading.

## Build and test

```
dotnet build OtterLogic.slnx
dotnet test OtterLogic.slnx
```

Arithmetic over coordinate arrays — no Rhino, no licence needed, runs anywhere.

Clone [Core](https://github.com/Otter-Logic/Core),
[Graphs](https://github.com/Otter-Logic/Graphs),
[MachineLearning](https://github.com/Otter-Logic/MachineLearning) and
[Unsupervised](https://github.com/Otter-Logic/Unsupervised) as sibling folders
and the project references resolve against your working copy; without them the
build falls back to the published packages.

## License

[MIT](LICENSE).
