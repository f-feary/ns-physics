# ns-physics

`ns-physics` is a performant Golang physics engine that uses a dynamic AABB tree for broadphase collision detection and supports various collider types, including boxes, spheres, and meshes. It is designed for use in real-time applications such as games and simulations.

## Features

- **Dynamic AABB Tree**: Efficient broadphase collision detection.
- **Collider Types**: Support for box, sphere, and mesh colliders.
- **Raycasting**: Perform raycasts to detect intersections with colliders.
- **Overlap Queries**: Detect overlapping objects using AABBs.
- **Point Containment Queries**: Check if a point is contained within any collider.
- **Transformable Colliders**: Apply transformations to colliders and update their bounds dynamically.
- **Benchmarking**: Includes benchmarks for performance testing.

## Limitations

`ns-physics` was created purely for server-side raycasting and overlapping usage, not as a real-time physics simulation. **ns-physics does not have any physics simulation capabilities**. If you decide to add simulation capabilities, please feel free to submit a pull request.

## Installation

To use `ns-physics`, you need to have Go installed. You can install the library by running:

```bash
go get github.com/f-feary/ns-physics
```

## Usage

### Creating a Scene

```go
import (
	"github.com/go-gl/mathgl/mgl32"
	"github.com/f-feary/ns-physics"
)

func main() {
	scene := physics.Scene{Tree: physics.DynamicAABBTree{}}

	// Add a BoxCollider
	boxNode := physics.NewSceneNode(1)
	boxNode.AddBoxCollider(
		physics.AABB{Min: mgl32.Vec3{-1, -1, -1}, Max: mgl32.Vec3{1, 1, 1}},
		mgl32.Ident4(),
	)
	scene.Tree.Insert(boxNode)

	// Add a SphereCollider
	sphereNode := physics.NewSceneNode(1)
	sphereNode.AddSphereCollider(mgl32.Vec3{5, 0, 0}, 1)
	scene.Tree.Insert(sphereNode)
}
```

### Performing a Raycast

```go
ray := physics.Ray{
	Origin:    mgl32.Vec3{-10, 0, 0},
	Direction: mgl32.Vec3{1, 0, 0},
	MaxLength: 20,
}
result := scene.Raycast(ray, 1)

if result != nil {
	fmt.Printf("Hit node ID: %d at distance: %f\n", result.Node.GetID(), result.Distance)
} else {
	fmt.Println("No hit detected")
}
```

### Overlap Query

```go
aabb := physics.AABB{
	Min: mgl32.Vec3{-2, -2, -2},
	Max: mgl32.Vec3{2, 2, 2},
}
results := scene.OverlapQuery(aabb, 1)

fmt.Printf("Found %d overlapping nodes\n", len(results))
```

### Point Containment Query

```go
point := mgl32.Vec3{0.5, 0.5, 0.5}
results := scene.PointContainmentQuery(point, 1)

fmt.Printf("Found %d nodes containing the point\n", len(results))
```

## Benchmarks

Run the included benchmarks to test the performance of the library:

```bash
go test -bench=.
```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.