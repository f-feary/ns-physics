package physics

import (
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/go-gl/mathgl/mgl32"
)

func TestRaycastClosestHit(t *testing.T) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Add a BoxCollider
	boxNode := NewSceneNode(1)
	boxNode.AddBoxCollider(
		AABB{Min: mgl32.Vec3{-1, -1, -1}, Max: mgl32.Vec3{1, 1, 1}},
	)
	scene.Tree.Insert(boxNode)

	// Add a SphereCollider
	sphereNode := NewSceneNode(1)
	sphereNode.AddSphereCollider(mgl32.Vec3{5, 0, 0}, 1)
	scene.Tree.Insert(sphereNode)

	// Perform a raycast
	ray := Ray{
		Origin:    mgl32.Vec3{-10, 0, 0},
		Direction: mgl32.Vec3{1, 0, 0},
		MaxLength: 20,
	}
	result := scene.Raycast(ray, 1)

	if result == nil {
		t.Fatalf("Expected a hit, but got none")
	}

	if result.Node.GetID() != boxNode.GetID() {
		t.Errorf("Expected closest hit to be boxNode, but got node with ID %d", result.Node.GetID())
	}

	if math.Abs(float64(result.Distance-9)) > 1e-5 {
		t.Errorf("Expected distance to be 9, but got %f", result.Distance)
	}
}

func TestRaycastAnyHit(t *testing.T) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Add a BoxCollider
	boxNode := NewSceneNode(1)
	boxNode.AddBoxCollider(
		AABB{Min: mgl32.Vec3{-1, -1, -1}, Max: mgl32.Vec3{1, 1, 1}},
	)
	scene.Tree.Insert(boxNode)

	// Perform a raycast
	ray := Ray{
		Origin:    mgl32.Vec3{-10, 0, 0},
		Direction: mgl32.Vec3{1, 0, 0},
		MaxLength: 20,
	}
	result := scene.RaycastAny(ray, 1)

	if result == nil {
		t.Fatalf("Expected a hit, but got none")
	}
}

func TestOverlapQuery(t *testing.T) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Add a BoxCollider
	boxNode := NewSceneNode(1)
	boxNode.AddBoxCollider(
		AABB{Min: mgl32.Vec3{-1, -1, -1}, Max: mgl32.Vec3{1, 1, 1}},
	)
	scene.Tree.Insert(boxNode)

	// Perform an overlap query
	aabb := AABB{
		Min: mgl32.Vec3{-2, -2, -2},
		Max: mgl32.Vec3{2, 2, 2},
	}
	results := scene.OverlapQuery(aabb, 1)

	if len(results) != 1 {
		t.Fatalf("Expected 1 overlapping node, but got %d", len(results))
	}

	if results[0].GetID() != boxNode.GetID() {
		t.Errorf("Expected overlapping node to be boxNode, but got node with ID %d", results[0].GetID())
	}
}

func TestPointContainmentQuery(t *testing.T) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Add a SphereCollider
	sphereNode := NewSceneNode(1)
	sphereNode.AddSphereCollider(mgl32.Vec3{0, 0, 0}, 1)
	scene.Tree.Insert(sphereNode)

	// Perform a point containment query
	point := mgl32.Vec3{0.5, 0.5, 0.5}
	results := scene.PointContainmentQuery(point, 1)

	if len(results) != 1 {
		t.Fatalf("Expected 1 containing node, but got %d", len(results))
	}

	if results[0].GetID() != sphereNode.GetID() {
		t.Errorf("Expected containing node to be sphereNode, but got node with ID %d", results[0].GetID())
	}
}

func TestBoxColliderTransform(t *testing.T) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Create a SceneNode with a BoxCollider and apply a translation transform
	boxNode := NewSceneNode(1)
	boxNode.Transform = mgl32.Translate3D(5, 0, 0) // Translate the box by (5, 0, 0)
	boxNode.AddBoxCollider(
		AABB{Min: mgl32.Vec3{-1, -1, -1}, Max: mgl32.Vec3{1, 1, 1}},
	)
	scene.Tree.Insert(boxNode)

	// Perform a raycast that should hit the translated box
	ray := Ray{
		Origin:    mgl32.Vec3{0, 0, 0},
		Direction: mgl32.Vec3{1, 0, 0},
		MaxLength: 10,
	}
	result := scene.Raycast(ray, 1)

	if result == nil {
		t.Fatalf("Expected a hit, but got none")
	}

	if result.Node.GetID() != boxNode.GetID() {
		t.Errorf("Expected hit to be boxNode, but got node with ID %d", result.Node.GetID())
	}

	if math.Abs(float64(result.Distance-4)) > 1e-5 {
		t.Errorf("Expected distance to be 4, but got %f", result.Distance)
	}
}

func TestSphereColliderTransform(t *testing.T) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Create a SceneNode with a SphereCollider and apply a translation transform
	sphereNode := NewSceneNode(1)
	sphereNode.Transform = mgl32.Translate3D(0, 5, 0) // Translate the sphere by (0, 5, 0)
	sphereNode.AddSphereCollider(mgl32.Vec3{0, 0, 0}, 1)
	scene.Tree.Insert(sphereNode)

	// Perform a raycast that should hit the translated sphere
	ray := Ray{
		Origin:    mgl32.Vec3{0, 0, 0},
		Direction: mgl32.Vec3{0, 1, 0},
		MaxLength: 10,
	}
	result := scene.Raycast(ray, 1)

	if result == nil {
		t.Fatalf("Expected a hit, but got none")
	}

	if result.Node.GetID() != sphereNode.GetID() {
		t.Errorf("Expected hit to be sphereNode, but got node with ID %d", result.Node.GetID())
	}

	if math.Abs(float64(result.Distance-4)) > 1e-5 {
		t.Errorf("Expected distance to be 4, but got %f", result.Distance)
	}
}

func TestNoOverlapQuery(t *testing.T) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Add a BoxCollider far from the query AABB
	boxNode := NewSceneNode(1)
	boxNode.AddBoxCollider(
		AABB{Min: mgl32.Vec3{100, 100, 100}, Max: mgl32.Vec3{110, 110, 110}},
	)
	scene.Tree.Insert(boxNode)

	// Perform an overlap query with an AABB that does not overlap
	aabb := AABB{
		Min: mgl32.Vec3{-10, -10, -10},
		Max: mgl32.Vec3{-5, -5, -5},
	}
	results := scene.OverlapQuery(aabb, 1)

	if len(results) != 0 {
		t.Fatalf("Expected no overlapping nodes, but got %d", len(results))
	}
}

func TestNoPointContainmentQuery(t *testing.T) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Add a SphereCollider far from the query point
	sphereNode := NewSceneNode(1)
	sphereNode.AddSphereCollider(mgl32.Vec3{100, 100, 100}, 5)
	scene.Tree.Insert(sphereNode)

	// Perform a point containment query with a point far from the sphere
	point := mgl32.Vec3{-10, -10, -10}
	results := scene.PointContainmentQuery(point, 1)

	if len(results) != 0 {
		t.Fatalf("Expected no containing nodes, but got %d", len(results))
	}
}

func TestNoRaycastHit(t *testing.T) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Add a BoxCollider far from the ray
	boxNode := NewSceneNode(1)
	boxNode.AddBoxCollider(
		AABB{Min: mgl32.Vec3{100, 100, 100}, Max: mgl32.Vec3{110, 110, 110}},
	)
	scene.Tree.Insert(boxNode)

	// Perform a raycast that does not intersect the box
	ray := Ray{
		Origin:    mgl32.Vec3{-10, -10, -10},
		Direction: mgl32.Vec3{1, 0, 0},
		MaxLength: 20,
	}
	result := scene.Raycast(ray, 1)

	if result != nil {
		t.Fatalf("Expected no hit, but got a hit with node ID %d", result.Node.GetID())
	}
}

func setupRaycastAny(scene *Scene, numColliders int) {
	for i := 0; i < numColliders; i++ {
		boxNode := NewSceneNode(1)
		boxNode.Transform = mgl32.Translate3D(
			rand.Float32()*1000, // Random x position
			rand.Float32()*50,   // Random y position
			rand.Float32()*1000, // Random z position
		)
		boxNode.AddBoxCollider(
			AABB{Min: mgl32.Vec3{-1, -1, -1}, Max: mgl32.Vec3{1, 1, 1}},
		)
		scene.Tree.Insert(boxNode)
	}
}

func runRaycastAny(scene *Scene, maxLength float32, numCasts int) {
	for i := 0; i < numCasts; i++ {
		ray := Ray{
			Origin:    mgl32.Vec3{rand.Float32() * 1000, rand.Float32() * 50, rand.Float32() * 1000},            // Random origin
			Direction: mgl32.Vec3{rand.Float32()*2 - 1, rand.Float32()*2 - 1, rand.Float32()*2 - 1}.Normalize(), // Random normalized direction
			MaxLength: maxLength,
		}
		_ = scene.RaycastAny(ray, 1)
	}
}

func BenchmarkRaycastAny50WithRandomizedScene1000(b *testing.B) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Seed the random number generator
	rand.Seed(time.Now().UnixNano())

	// Add 1000 BoxColliders to the scene with random positions
	setupRaycastAny(&scene, 1000)

	// Benchmark the RaycastAny method with random rays
	b.ResetTimer()

	runRaycastAny(&scene, 50.0, b.N)

}

func BenchmarkRaycastAny1WithRandomizedScene1000(b *testing.B) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Seed the random number generator
	rand.Seed(time.Now().UnixNano())

	// Add 1000 BoxColliders to the scene with random positions
	setupRaycastAny(&scene, 1000)

	// Benchmark the RaycastAny method with random rays
	b.ResetTimer()

	runRaycastAny(&scene, 1.5, b.N)

}

func BenchmarkRaycastAny1WithRandomizedScene10000(b *testing.B) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Seed the random number generator
	rand.Seed(time.Now().UnixNano())

	// Add 1000 BoxColliders to the scene with random positions
	setupRaycastAny(&scene, 10000)

	// Benchmark the RaycastAny method with random rays
	b.ResetTimer()

	runRaycastAny(&scene, 1.5, b.N)

}

func BenchmarkUpdateSingleNodeInScene10000(b *testing.B) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Seed the random number generator
	rand.Seed(time.Now().UnixNano())

	// Add 10,000 static BoxColliders to the scene
	setupRaycastAny(&scene, 10000)

	// Add a single dynamic node to the scene
	dynamicNode := NewSceneNode(1)
	dynamicNode.AddBoxCollider(
		AABB{Min: mgl32.Vec3{-1, -1, -1}, Max: mgl32.Vec3{1, 1, 1}},
	)
	scene.Tree.Insert(dynamicNode)

	// Benchmark the update of the dynamic node's position
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		// Randomly move the dynamic node
		dynamicNode.Transform = mgl32.Translate3D(
			rand.Float32()*1000, // Random x position
			rand.Float32()*50,   // Random y position
			rand.Float32()*1000, // Random z position
		)
		dynamicNode.MarkedForUpdate = true

		// Update the tree
		scene.Tree.UpdateTree()
	}
}

func TestIntersectAABBOnSphereCollider(t *testing.T) {
	sphere := SphereCollider{
		Center: mgl32.Vec3{0, 0, 0},
		Radius: 1,
	}
	aabb := AABB{
		Min: mgl32.Vec3{-0.5, -0.5, -0.5},
		Max: mgl32.Vec3{0.5, 0.5, 0.5},
	}
	if !sphere.IntersectAABB(aabb) {
		t.Fatalf("Expected SphereCollider to intersect with AABB")
	}
}

func TestIntersectAABBOnBoxCollider(t *testing.T) {
	box := BoxCollider{
		LocalBounds: AABB{
			Min: mgl32.Vec3{-1, -1, -1},
			Max: mgl32.Vec3{1, 1, 1},
		},
		WorldTransform: nil,
	}
	aabb := AABB{
		Min: mgl32.Vec3{-0.5, -0.5, -0.5},
		Max: mgl32.Vec3{0.5, 0.5, 0.5},
	}
	if !box.IntersectAABB(aabb) {
		t.Fatalf("Expected BoxCollider to intersect with AABB")
	}
}

func TestSceneNodeWithMultipleColliders(t *testing.T) {
	node := NewSceneNode(1)
	node.AddBoxCollider(
		AABB{Min: mgl32.Vec3{-1, -1, -1}, Max: mgl32.Vec3{1, 1, 1}},
	)
	node.AddSphereCollider(mgl32.Vec3{5, 0, 0}, 1)

	aabb := AABB{
		Min: mgl32.Vec3{4, -1, -1},
		Max: mgl32.Vec3{6, 1, 1},
	}
	if !node.IntersectAABB(aabb) {
		t.Fatalf("Expected SceneNode with multiple colliders to intersect with AABB")
	}
}

func TestRemoveNode(t *testing.T) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Add two nodes
	node1 := NewSceneNode(1)
	node1.AddBoxCollider(
		AABB{Min: mgl32.Vec3{-1, -1, -1}, Max: mgl32.Vec3{1, 1, 1}},
	)
	scene.Tree.Insert(node1)

	node2 := NewSceneNode(1)
	node2.AddSphereCollider(mgl32.Vec3{5, 0, 0}, 1)
	scene.Tree.Insert(node2)

	// Perform a query before removal
	aabb := AABB{
		Min: mgl32.Vec3{-2, -2, -2},
		Max: mgl32.Vec3{2, 2, 2},
	}
	results := scene.OverlapQuery(aabb, 1)
	if len(results) != 1 {
		t.Fatalf("Expected 1 overlapping node before removal, but got %d", len(results))
	}

	// Remove node1
	scene.Tree.Remove(node1)

	// Perform a query after removal
	results = scene.OverlapQuery(aabb, 1)
	if len(results) != 0 {
		t.Fatalf("Expected no overlapping nodes after removal, but got %d", len(results))
	}
}

func TestUpdateTree(t *testing.T) {
	scene := Scene{Tree: DynamicAABBTree{}}

	// Add a dynamic node
	node := NewSceneNode(1)
	node.AddBoxCollider(
		AABB{Min: mgl32.Vec3{-1, -1, -1}, Max: mgl32.Vec3{1, 1, 1}},
	)
	scene.Tree.Insert(node)

	// Perform a query before moving the node
	aabb := AABB{
		Min: mgl32.Vec3{-2, -2, -2},
		Max: mgl32.Vec3{2, 2, 2},
	}
	results := scene.OverlapQuery(aabb, 1)
	if len(results) != 1 {
		t.Fatalf("Expected 1 overlapping node before moving, but got %d", len(results))
	}

	// Move the node and update the tree
	node.Transform = mgl32.Translate3D(10, 0, 0)
	node.MarkedForUpdate = true
	scene.Tree.UpdateTree()

	// Perform a query after moving the node
	results = scene.OverlapQuery(aabb, 1)
	if len(results) != 0 {
		t.Fatalf("Expected no overlapping nodes after moving, but got %d", len(results))
	}

	// Query the new position
	newAABB := AABB{
		Min: mgl32.Vec3{9, -2, -2},
		Max: mgl32.Vec3{11, 2, 2},
	}
	results = scene.OverlapQuery(newAABB, 1)
	if len(results) != 1 {
		t.Fatalf("Expected 1 overlapping node at new position, but got %d", len(results))
	}
}
