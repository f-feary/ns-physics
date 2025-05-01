package physics

import (
	"math"
	"sort"

	"github.com/go-gl/mathgl/mgl32"
)

// Triangle represents a single triangle in the mesh.
type Triangle struct {
	Vertices [3]mgl32.Vec3
}

// BVHNode represents a node in the Bounding Volume Hierarchy.
type BVHNode struct {
	Bounds    AABB
	Left      *BVHNode
	Right     *BVHNode
	IsLeaf    bool
	Triangles []int // Indices of triangles for leaf nodes
}

// MeshCollider represents a collider for a triangle mesh with a BVH for acceleration.
type MeshCollider struct {
	Triangles            []Triangle
	TransformedTriangles []Triangle // Cache for transformed triangles
	BVHRoot              *BVHNode
	WorldTransform       *mgl32.Mat4
}

// NewMeshCollider creates a new MeshCollider and builds the BVH.
func NewMeshCollider(triangles []Triangle, worldTransform *mgl32.Mat4) *MeshCollider {
	collider := &MeshCollider{
		Triangles:            triangles,
		WorldTransform:       worldTransform,
		TransformedTriangles: make([]Triangle, len(triangles)),
	}
	collider.updateTransformedTriangles()
	collider.BVHRoot = buildBVH(collider.TransformedTriangles, 0, len(collider.TransformedTriangles))
	return collider
}

// updateTransformedTriangles updates the cached transformed triangles based on the WorldTransform.
func (m *MeshCollider) updateTransformedTriangles() {
	if m.WorldTransform == nil {
		copy(m.TransformedTriangles, m.Triangles)
		return
	}

	for i, triangle := range m.Triangles {
		m.TransformedTriangles[i] = Triangle{
			Vertices: [3]mgl32.Vec3{
				mgl32.TransformCoordinate(triangle.Vertices[0], *m.WorldTransform),
				mgl32.TransformCoordinate(triangle.Vertices[1], *m.WorldTransform),
				mgl32.TransformCoordinate(triangle.Vertices[2], *m.WorldTransform),
			},
		}
	}
}

// GetAABB computes the world-space AABB of the MeshCollider.
func (m *MeshCollider) GetAABB() AABB {
	if m.WorldTransform != nil {
		return transformAABB(m.BVHRoot.Bounds, *m.WorldTransform)
	}
	return m.BVHRoot.Bounds
}

// IntersectRay performs a raycast against the mesh using the BVH for acceleration.
func (m *MeshCollider) IntersectRay(ray Ray) (bool, float32) {
	hit, distance := intersectRayBVH(ray, m.BVHRoot, m.TransformedTriangles)
	if hit && (ray.MaxLength == 0 || distance <= ray.MaxLength) {
		return true, distance
	}
	return false, 0
}

// IntersectAABB checks if the mesh intersects with an AABB using the BVH.
func (m *MeshCollider) IntersectAABB(aabb AABB) bool {
	return intersectAABBBVH(aabb, m.BVHRoot)
}

// buildBVH recursively builds the BVH for the given triangles.
func buildBVH(triangles []Triangle, start, end int) *BVHNode {
	if start >= end {
		return nil
	}

	// Compute the AABB for the current set of triangles.
	bounds := computeAABB(triangles[start:end])

	// If this is a leaf node, store the triangle indices.
	if end-start <= 2 {
		return &BVHNode{
			Bounds:    bounds,
			IsLeaf:    true,
			Triangles: makeRange(start, end),
		}
	}

	// Split the triangles along the largest axis of the AABB.
	axis := largestAxis(bounds)
	mid := (start + end) / 2
	sortTriangles(triangles[start:end], axis)

	// Recursively build child nodes.
	left := buildBVH(triangles, start, mid)
	right := buildBVH(triangles, mid, end)

	return &BVHNode{
		Bounds: mergeAABB(left.Bounds, right.Bounds),
		Left:   left,
		Right:  right,
	}
}

// intersectRayBVH performs a raycast against the BVH.
func intersectRayBVH(ray Ray, node *BVHNode, triangles []Triangle) (bool, float32) {
	if node == nil {
		return false, 0
	}

	// Check if the ray intersects the node's AABB.
	hit, _ := intersectRayAABB(ray, node.Bounds)
	if !hit {
		return false, 0
	}

	// If this is a leaf node, test the triangles.
	if node.IsLeaf {
		var closestHit bool
		var closestDistance float32 = math.MaxFloat32
		for _, triIndex := range node.Triangles {
			if hit, distance := intersectRayTriangle(ray, triangles[triIndex]); hit && distance < closestDistance {
				closestHit = true
				closestDistance = distance
			}
		}
		return closestHit, closestDistance
	}

	// Recursively test child nodes.
	hitLeft, distLeft := intersectRayBVH(ray, node.Left, triangles)
	hitRight, distRight := intersectRayBVH(ray, node.Right, triangles)

	if hitLeft && (!hitRight || distLeft < distRight) {
		return hitLeft, distLeft
	}
	return hitRight, distRight
}

// intersectAABBBVH checks if an AABB intersects the BVH.
func intersectAABBBVH(aabb AABB, node *BVHNode) bool {
	if node == nil {
		return false
	}

	// Check if the AABBs overlap.
	if !intersectAABB(aabb, node.Bounds) {
		return false
	}

	// If this is a leaf node, return true.
	if node.IsLeaf {
		return true
	}

	// Recursively check child nodes.
	return intersectAABBBVH(aabb, node.Left) || intersectAABBBVH(aabb, node.Right)
}

// intersectRayTriangle performs a ray-triangle intersection test.
func intersectRayTriangle(ray Ray, triangle Triangle) (bool, float32) {
	// Möller-Trumbore intersection algorithm.
	e1 := triangle.Vertices[1].Sub(triangle.Vertices[0])
	e2 := triangle.Vertices[2].Sub(triangle.Vertices[0])
	h := ray.Direction.Cross(e2)
	a := e1.Dot(h)

	if a > -1e-5 && a < 1e-5 {
		return false, 0 // Ray is parallel to the triangle.
	}

	f := 1.0 / a
	s := ray.Origin.Sub(triangle.Vertices[0])
	u := f * s.Dot(h)

	if u < 0.0 || u > 1.0 {
		return false, 0
	}

	q := s.Cross(e1)
	v := f * ray.Direction.Dot(q)

	if v < 0.0 || u+v > 1.0 {
		return false, 0
	}

	t := f * e2.Dot(q)
	if t > 1e-5 && (ray.MaxLength == 0 || t <= ray.MaxLength) {
		return true, t
	}

	return false, 0
}

// transformAABB applies a transformation matrix to an AABB.
func transformAABB(aabb AABB, transform mgl32.Mat4) AABB {
	corners := []mgl32.Vec3{
		aabb.Min,
		{aabb.Max.X(), aabb.Min.Y(), aabb.Min.Z()},
		{aabb.Min.X(), aabb.Max.Y(), aabb.Min.Z()},
		{aabb.Min.X(), aabb.Min.Y(), aabb.Max.Z()},
		{aabb.Max.X(), aabb.Max.Y(), aabb.Min.Z()},
		{aabb.Max.X(), aabb.Min.Y(), aabb.Max.Z()},
		{aabb.Min.X(), aabb.Max.Y(), aabb.Max.Z()},
		aabb.Max,
	}

	var transformedCorners []mgl32.Vec3
	for _, corner := range corners {
		transformedCorners = append(transformedCorners, mgl32.TransformCoordinate(corner, transform))
	}

	min := transformedCorners[0]
	max := transformedCorners[0]
	for _, corner := range transformedCorners {
		min = mgl32.Vec3{
			minComponent(min.X(), corner.X()),
			minComponent(min.Y(), corner.Y()),
			minComponent(min.Z(), corner.Z()),
		}
		max = mgl32.Vec3{
			maxComponent(max.X(), corner.X()),
			maxComponent(max.Y(), corner.Y()),
			maxComponent(max.Z(), corner.Z()),
		}
	}

	return AABB{Min: min, Max: max}
}

// computeAABB computes the AABB for a set of triangles.
func computeAABB(triangles []Triangle) AABB {
	min := triangles[0].Vertices[0]
	max := triangles[0].Vertices[0]

	for _, tri := range triangles {
		for _, vertex := range tri.Vertices {
			min = mgl32.Vec3{
				minComponent(min.X(), vertex.X()),
				minComponent(min.Y(), vertex.Y()),
				minComponent(min.Z(), vertex.Z()),
			}
			max = mgl32.Vec3{
				maxComponent(max.X(), vertex.X()),
				maxComponent(max.Y(), vertex.Y()),
				maxComponent(max.Z(), vertex.Z()),
			}
		}
	}

	return AABB{Min: min, Max: max}
}

// Helper functions for BVH construction and traversal.
func largestAxis(aabb AABB) int {
	size := aabb.Max.Sub(aabb.Min)
	if size.X() > size.Y() && size.X() > size.Z() {
		return 0
	} else if size.Y() > size.Z() {
		return 1
	}
	return 2
}

func sortTriangles(triangles []Triangle, axis int) {
	// Sort triangles based on the centroid along the specified axis.
	sort.Slice(triangles, func(i, j int) bool {
		centroidI := triangles[i].Vertices[0].Add(triangles[i].Vertices[1]).Add(triangles[i].Vertices[2]).Mul(1.0 / 3.0)
		centroidJ := triangles[j].Vertices[0].Add(triangles[j].Vertices[1]).Add(triangles[j].Vertices[2]).Mul(1.0 / 3.0)
		return centroidI[axis] < centroidJ[axis]
	})
}

func makeRange(start, end int) []int {
	r := make([]int, end-start)
	for i := range r {
		r[i] = start + i
	}
	return r
}
