package physics

import (
	"testing"

	"github.com/go-gl/mathgl/mgl32"
)

// Hardcoded cube mesh (unit cube centered at origin)
var cubeTriangles = []Triangle{
	// Front face
	{Vertices: [3]mgl32.Vec3{{-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}}},
	{Vertices: [3]mgl32.Vec3{{-0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {-0.5, 0.5, 0.5}}},
	// Back face
	{Vertices: [3]mgl32.Vec3{{-0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {0.5, -0.5, -0.5}}},
	{Vertices: [3]mgl32.Vec3{{-0.5, -0.5, -0.5}, {-0.5, 0.5, -0.5}, {0.5, 0.5, -0.5}}},
	// Left face
	{Vertices: [3]mgl32.Vec3{{-0.5, -0.5, -0.5}, {-0.5, -0.5, 0.5}, {-0.5, 0.5, 0.5}}},
	{Vertices: [3]mgl32.Vec3{{-0.5, -0.5, -0.5}, {-0.5, 0.5, 0.5}, {-0.5, 0.5, -0.5}}},
	// Right face
	{Vertices: [3]mgl32.Vec3{{0.5, -0.5, -0.5}, {0.5, 0.5, 0.5}, {0.5, -0.5, 0.5}}},
	{Vertices: [3]mgl32.Vec3{{0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {0.5, 0.5, 0.5}}},
	// Top face
	{Vertices: [3]mgl32.Vec3{{-0.5, 0.5, -0.5}, {-0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}}},
	{Vertices: [3]mgl32.Vec3{{-0.5, 0.5, -0.5}, {0.5, 0.5, 0.5}, {0.5, 0.5, -0.5}}},
	// Bottom face
	{Vertices: [3]mgl32.Vec3{{-0.5, -0.5, -0.5}, {0.5, -0.5, 0.5}, {-0.5, -0.5, 0.5}}},
	{Vertices: [3]mgl32.Vec3{{-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, -0.5, 0.5}}},
}

func TestMeshColliderRaycast(t *testing.T) {
	// Create a MeshCollider with the cube mesh
	meshCollider := NewMeshCollider(cubeTriangles, nil)

	// Perform a raycast
	ray := Ray{
		Origin:    mgl32.Vec3{0, 0, -1}, // Start outside the cube
		Direction: mgl32.Vec3{0, 0, 1},  // Pointing towards the cube
		MaxLength: 10,
	}
	hit, distance := meshCollider.IntersectRay(ray)

	// Validate the results
	if !hit {
		t.Fatalf("Expected ray to hit the cube, but it did not")
	}

	expectedDistance := float32(0.5) // Distance to the front face of the cube
	if abs(distance-expectedDistance) > 1e-5 {
		t.Errorf("Expected hit distance to be %f, but got %f", expectedDistance, distance)
	}
}

func TestMeshColliderRaycastWithTransformHit(t *testing.T) {
	// Apply a translation transform to move the cube to (2, 0, 0)
	worldTransform := mgl32.Translate3D(2, 0, 0)

	// Create a MeshCollider with the cube mesh and transform
	meshCollider := NewMeshCollider(cubeTriangles, &worldTransform)

	// Perform a raycast
	ray := Ray{
		Origin:    mgl32.Vec3{0, 0, 0}, // Start at the origin
		Direction: mgl32.Vec3{1, 0, 0}, // Pointing towards the cube
		MaxLength: 10,
	}
	hit, distance := meshCollider.IntersectRay(ray)

	// Validate the results
	if !hit {
		t.Fatalf("Expected ray to hit the transformed cube, but it did not")
	}

	expectedDistance := float32(1.5) // Distance to the front face of the transformed cube
	if abs(distance-expectedDistance) > 1e-5 {
		t.Errorf("Expected hit distance to be %f, but got %f", expectedDistance, distance)
	}
}

func TestMeshColliderRaycastWithTransformNoHit(t *testing.T) {

	// Apply a translation transform to move the cube far away
	worldTransform := mgl32.Translate3D(10, 0, 0)

	// Create a MeshCollider with the cube mesh and transform
	meshCollider := NewMeshCollider(cubeTriangles, &worldTransform)

	// Perform a raycast
	ray := Ray{
		Origin:    mgl32.Vec3{0, 0, 0}, // Start at the origin
		Direction: mgl32.Vec3{1, 0, 0}, // Pointing towards where the cube would have been
		MaxLength: 5,
	}
	hit, distance := meshCollider.IntersectRay(ray)

	// Validate the results
	if hit {
		t.Fatalf("Expected ray to miss the transformed cube, but it hit at %f", distance)
	}
}

// abs computes the absolute value of a float32.
func abs(x float32) float32 {
	if x < 0 {
		return -x
	}
	return x
}
