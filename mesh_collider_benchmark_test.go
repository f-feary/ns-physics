package physics

import (
	"math/rand"
	"testing"
	"time"

	"github.com/go-gl/mathgl/mgl32"
)

// BenchmarkMeshColliderRaycast benchmarks raycast performance with 1000 randomly placed MeshColliders.
func BenchmarkMeshColliderRaycast(b *testing.B) {
	// Seed the random number generator
	rand.Seed(time.Now().UnixNano())

	// Create a scene and add 1000 randomly placed MeshColliders
	scene := Scene{Tree: DynamicAABBTree{}}
	for i := 0; i < 1000; i++ {
		transform := mgl32.Translate3D(
			rand.Float32()*1000, // Random x position
			rand.Float32()*50,   // Random y position
			rand.Float32()*1000, // Random z position
		)
		meshCollider := NewMeshCollider(cubeTriangles, &transform)
		node := NewSceneNode(1)
		node.Colliders = append(node.Colliders, meshCollider)
		node.UpdateBounds()
		scene.Tree.Insert(node)
	}

	// Benchmark the raycast performance
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		ray := Ray{
			Origin:    mgl32.Vec3{0, 0, 0},                                                                      // Start at the origin
			Direction: mgl32.Vec3{rand.Float32()*2 - 1, rand.Float32()*2 - 1, rand.Float32()*2 - 1}.Normalize(), // Random direction
			MaxLength: 500,                                                                                      // Maximum ray length
		}
		scene.Raycast(ray, 1)
	}
}
