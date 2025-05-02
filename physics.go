package physics

import (
	"math"        // Add the math package for square root operations
	"sync/atomic" // For atomic counter

	"github.com/go-gl/mathgl/mgl32"
)

var nodeIDCounter uint64 // Global atomic counter for SceneNode IDs

// Ray represents a 3D ray with an origin, direction, and optional maximum length.
type Ray struct {
	Origin    mgl32.Vec3
	Direction mgl32.Vec3
	MaxLength float32 // Maximum length of the ray (0 means infinite)
}

// AABB represents an axis-aligned bounding box.
type AABB struct {
	Min mgl32.Vec3
	Max mgl32.Vec3
}

// Collider represents an abstract interface for objects that can be queried in the scene.
type Collider interface {
	GetAABB() AABB
	IntersectRay(ray Ray) (bool, float32) // Returns whether the ray intersects and the distance
	IntersectAABB(aabb AABB) bool         // Returns whether the collider intersects with an AABB
}

// BoxCollider is a concrete implementation of Collider for a transformable 3D cube.
type BoxCollider struct {
	LocalBounds    AABB        // Local-space AABB
	WorldTransform *mgl32.Mat4 // Pointer to the SceneNode's world transform
}

// GetAABB computes and returns the world-space AABB of the BoxCollider.
func (b *BoxCollider) GetAABB() AABB {
	// Use only the world transform to compute the final transform
	finalTransform := mgl32.Ident4()
	if b.WorldTransform != nil {
		finalTransform = *b.WorldTransform
	}

	// Extract the 8 corners of the local AABB
	corners := []mgl32.Vec3{
		b.LocalBounds.Min,
		{b.LocalBounds.Max.X(), b.LocalBounds.Min.Y(), b.LocalBounds.Min.Z()},
		{b.LocalBounds.Min.X(), b.LocalBounds.Max.Y(), b.LocalBounds.Min.Z()},
		{b.LocalBounds.Min.X(), b.LocalBounds.Min.Y(), b.LocalBounds.Max.Z()},
		{b.LocalBounds.Max.X(), b.LocalBounds.Max.Y(), b.LocalBounds.Min.Z()},
		{b.LocalBounds.Max.X(), b.LocalBounds.Min.Y(), b.LocalBounds.Max.Z()},
		{b.LocalBounds.Min.X(), b.LocalBounds.Max.Y(), b.LocalBounds.Max.Z()},
		b.LocalBounds.Max,
	}

	// Transform the corners to world space
	var transformedCorners []mgl32.Vec3
	for _, corner := range corners {
		transformedCorners = append(transformedCorners, mgl32.TransformCoordinate(corner, finalTransform))
	}

	// Compute the world-space AABB from the transformed corners
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

// IntersectRay checks if a ray intersects the BoxCollider's computed AABB.
func (b *BoxCollider) IntersectRay(ray Ray) (bool, float32) {
	return intersectRayAABB(ray, b.GetAABB())
}

// IntersectAABB checks if the BoxCollider's computed AABB intersects another AABB.
func (b *BoxCollider) IntersectAABB(aabb AABB) bool {
	return intersectAABB(b.GetAABB(), aabb)
}

// SphereCollider is a concrete implementation of Collider for a sphere.
type SphereCollider struct {
	Center         mgl32.Vec3  // Local-space center of the sphere
	Radius         float32     // Radius of the sphere
	WorldTransform *mgl32.Mat4 // Pointer to the SceneNode's world transform
}

// GetAABB computes and returns the world-space AABB of the SphereCollider.
func (s *SphereCollider) GetAABB() AABB {
	// Transform the center to world space
	worldCenter := s.Center
	if s.WorldTransform != nil {
		worldCenter = mgl32.TransformCoordinate(s.Center, *s.WorldTransform)
	}

	// Compute the AABB from the transformed center and radius
	radiusVec := mgl32.Vec3{s.Radius, s.Radius, s.Radius}
	return AABB{
		Min: worldCenter.Sub(radiusVec),
		Max: worldCenter.Add(radiusVec),
	}
}

// IntersectRay checks if a ray intersects the SphereCollider.
func (s *SphereCollider) IntersectRay(ray Ray) (bool, float32) {
	// Transform the center to world space
	worldCenter := s.Center
	if s.WorldTransform != nil {
		worldCenter = mgl32.TransformCoordinate(s.Center, *s.WorldTransform)
	}

	// Perform ray-sphere intersection test
	oc := ray.Origin.Sub(worldCenter)
	a := ray.Direction.Dot(ray.Direction)
	b := 2.0 * oc.Dot(ray.Direction)
	c := oc.Dot(oc) - s.Radius*s.Radius
	discriminant := b*b - 4*a*c

	if discriminant < 0 {
		return false, 0
	}
	discRoot := float32(math.Sqrt(float64(discriminant)))
	// Compute the nearest intersection distance
	t1 := (-b - discRoot) / (2 * a)
	t2 := (-b + discRoot) / (2 * a)

	if t1 >= 0 {
		return true, t1
	} else if t2 >= 0 {
		return true, t2
	}

	return false, 0
}

// IntersectAABB checks if the SphereCollider intersects with an AABB.
func (s *SphereCollider) IntersectAABB(aabb AABB) bool {
	// Transform the center to world space
	worldCenter := s.Center
	if s.WorldTransform != nil {
		worldCenter = mgl32.TransformCoordinate(s.Center, *s.WorldTransform)
	}

	// Find the closest point on the AABB to the sphere's center
	closestPoint := mgl32.Vec3{
		clamp(worldCenter.X(), aabb.Min.X(), aabb.Max.X()),
		clamp(worldCenter.Y(), aabb.Min.Y(), aabb.Max.Y()),
		clamp(worldCenter.Z(), aabb.Min.Z(), aabb.Max.Z()),
	}

	// Check if the distance from the closest point to the sphere's center is less than the radius
	distanceSquared := closestPoint.Sub(worldCenter).LenSqr()
	return distanceSquared <= s.Radius*s.Radius
}

// Helper function to clamp a value between a minimum and maximum.
func clamp(value, min, max float32) float32 {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}

// Helper functions for min and max components
func minComponent(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

func maxComponent(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}

// SceneNode now supports an immutable ID for equality checks.
type SceneNode struct {
	id              uint64     // Immutable unique ID
	Transform       mgl32.Mat4 // World transform matrix
	Colliders       []Collider // List of colliders
	LayerMask       uint32     // Layer mask for filtering
	Bounds          AABB       // Cached world-space AABB
	IsStatic        bool
	MarkedForUpdate bool // Indicates whether a SceneNode needs to be updated in the tree
}

// Updateable represents an interface for colliders that require updates.
type Updateable interface {
	Update()
}

// NewSceneNode creates a new SceneNode with a unique ID and the specified layer mask.
func NewSceneNode(layerMask uint32) *SceneNode {
	return &SceneNode{
		id:        atomic.AddUint64(&nodeIDCounter, 1),
		LayerMask: layerMask,
		Transform: mgl32.Ident4(), // Initialize to identity matrix
	}
}

// AddMeshCollider adds a MeshCollider to the SceneNode and updates its AABB.
func (node *SceneNode) AddMeshCollider(triangles []Triangle) {
	meshCollider := NewMeshCollider(triangles, &node.Transform)
	node.Colliders = append(node.Colliders, meshCollider)
	node.UpdateBounds()
}

// AddBoxCollider adds a BoxCollider to the SceneNode and updates its AABB.
func (node *SceneNode) AddBoxCollider(localBounds AABB) {
	boxCollider := &BoxCollider{
		LocalBounds:    localBounds,
		WorldTransform: &node.Transform,
	}
	node.Colliders = append(node.Colliders, boxCollider)
	node.UpdateBounds()
}

// AddSphereCollider adds a SphereCollider to the SceneNode and updates its AABB.
func (node *SceneNode) AddSphereCollider(center mgl32.Vec3, radius float32) {
	sphereCollider := &SphereCollider{
		Center:         center,
		Radius:         radius,
		WorldTransform: &node.Transform,
	}
	node.Colliders = append(node.Colliders, sphereCollider)
	node.UpdateBounds()
}

// GetID returns the immutable ID of the SceneNode.
func (node *SceneNode) GetID() uint64 {
	return node.id
}

// UpdateBounds updates the cached AABB of the SceneNode based on all its Colliders.
func (node *SceneNode) UpdateBounds() {
	if len(node.Colliders) == 0 {
		return
	}

	// Call Update() for colliders that implement the Updateable interface.
	for _, collider := range node.Colliders {
		if updateable, ok := collider.(Updateable); ok {
			updateable.Update()
		}
	}

	// Initialize bounds with the first collider's AABB.
	node.Bounds = node.Colliders[0].GetAABB()

	// Merge the bounds of all colliders.
	for _, collider := range node.Colliders[1:] {
		node.Bounds = mergeAABB(node.Bounds, collider.GetAABB())
	}
}

// Raycast performs a raycast against all colliders in the SceneNode and returns the closest hit.
func (node *SceneNode) Raycast(ray Ray) (bool, float32) {
	var closestHit bool
	var closestDistance float32 = math.MaxFloat32

	for _, collider := range node.Colliders {
		if hit, distance := collider.IntersectRay(ray); hit && (ray.MaxLength == 0 || distance <= ray.MaxLength) {
			if distance < closestDistance {
				closestHit = true
				closestDistance = distance
			}
		}
	}

	return closestHit, closestDistance
}

// IntersectAABB checks if any collider in the SceneNode intersects with the given AABB.
func (node *SceneNode) IntersectAABB(aabb AABB) bool {
	for _, collider := range node.Colliders {
		if collider.IntersectAABB(aabb) {
			return true
		}
	}
	return false
}

// DynamicAABBTreeNode represents a node in the dynamic AABB tree.
type DynamicAABBTreeNode struct {
	Bounds AABB
	Node   *SceneNode
	Left   *DynamicAABBTreeNode
	Right  *DynamicAABBTreeNode
	IsLeaf bool
}

// DynamicAABBTree represents the broadphase acceleration structure.
// The tree dynamically adjusts to changes in the scene, ensuring efficient queries
// even for dynamic objects.
type DynamicAABBTree struct {
	Root *DynamicAABBTreeNode
}

// Insert adds a new SceneNode to the tree.
func (tree *DynamicAABBTree) Insert(node *SceneNode) {
	if !node.IsStatic {
		node.UpdateBounds()
	}
	newNode := &DynamicAABBTreeNode{
		Bounds: node.Bounds,
		Node:   node,
		IsLeaf: true,
	}
	if tree.Root == nil {
		tree.Root = newNode
		return
	}
	tree.Root = insertNode(tree.Root, newNode)
}

// Remove removes a SceneNode from the tree.
func (tree *DynamicAABBTree) Remove(node *SceneNode) {
	if tree.Root == nil {
		return
	}
	tree.Root = removeNode(tree.Root, node)
}

// removeNode recursively removes a node from the tree.
func removeNode(root *DynamicAABBTreeNode, node *SceneNode) *DynamicAABBTreeNode {
	if root == nil {
		return nil
	}
	if root.IsLeaf && root.Node == node {
		return nil
	}
	root.Left = removeNode(root.Left, node)
	root.Right = removeNode(root.Right, node)
	if root.Left == nil && root.Right == nil {
		return nil
	}
	if root.Left == nil {
		return root.Right
	}
	if root.Right == nil {
		return root.Left
	}
	root.Bounds = mergeAABB(root.Left.Bounds, root.Right.Bounds)
	return root
}

// insertNode recursively inserts a node into the tree.
func insertNode(root, newNode *DynamicAABBTreeNode) *DynamicAABBTreeNode {
	if root.IsLeaf {
		parent := &DynamicAABBTreeNode{
			Bounds: mergeAABB(root.Bounds, newNode.Bounds),
			Left:   root,
			Right:  newNode,
			IsLeaf: false,
		}
		return parent
	}
	leftMerged := mergeAABB(root.Left.Bounds, newNode.Bounds)
	rightMerged := mergeAABB(root.Right.Bounds, newNode.Bounds)

	if volume(leftMerged) < volume(rightMerged) {
		root.Left = insertNode(root.Left, newNode)
		root.Bounds = mergeAABB(root.Left.Bounds, root.Right.Bounds)
	} else {
		root.Right = insertNode(root.Right, newNode)
		root.Bounds = mergeAABB(root.Left.Bounds, root.Right.Bounds)
	}
	return root
}

// UpdateTree incrementally updates the tree structure to reflect changes in node positions.
// This avoids rebuilding the entire tree, making it suitable for real-time applications.
func (tree *DynamicAABBTree) UpdateTree() {
	var updatedNodes []*SceneNode
	collectUpdatedNodes(tree.Root, &updatedNodes)

	for _, node := range updatedNodes {
		tree.Remove(node)
		node.UpdateBounds()
		node.MarkedForUpdate = false // Reset the update flag
		tree.Insert(node)
	}
}

// collectUpdatedNodes collects all nodes marked for update from the tree.
func collectUpdatedNodes(node *DynamicAABBTreeNode, updatedNodes *[]*SceneNode) {
	if node == nil {
		return
	}
	if node.IsLeaf && node.Node.MarkedForUpdate {
		*updatedNodes = append(*updatedNodes, node.Node)
		return
	}
	collectUpdatedNodes(node.Left, updatedNodes)
	collectUpdatedNodes(node.Right, updatedNodes)
}

// Query performs a broadphase query on the tree with a given AABB.
func (tree *DynamicAABBTree) Query(aabb AABB, mask uint32) []*SceneNode {
	var results []*SceneNode
	queryNode(tree.Root, aabb, mask, &results)
	return results
}

// queryNode recursively queries the tree for overlapping nodes.
func queryNode(node *DynamicAABBTreeNode, aabb AABB, mask uint32, results *[]*SceneNode) {
	if node == nil {
		return
	}
	if !intersectAABB(node.Bounds, aabb) {
		return
	}
	if node.IsLeaf && (node.Node.LayerMask&mask) != 0 {
		*results = append(*results, node.Node)
		return
	}
	queryNode(node.Left, aabb, mask, results)
	queryNode(node.Right, aabb, mask, results)
}

// Raycast performs a broadphase raycast query on the tree with a maximum ray length.
// By pruning branches that do not intersect the ray, this method reduces the number of narrowphase tests.
func (tree *DynamicAABBTree) Raycast(ray Ray, mask uint32) []*SceneNode {
	var results []*SceneNode
	raycastNode(tree.Root, ray, mask, &results)
	return results
}

// raycastNode recursively performs a raycast query on the tree, pruning branches based on ray length.
func raycastNode(node *DynamicAABBTreeNode, ray Ray, mask uint32, results *[]*SceneNode) {
	if node == nil {
		return
	}

	// Perform broadphase AABB-ray intersection test with pruning based on MaxLength
	hit, distance := intersectRayAABB(ray, node.Bounds)
	if !hit || (ray.MaxLength > 0 && distance > ray.MaxLength) {
		return
	}

	if node.IsLeaf && (node.Node.LayerMask&mask) != 0 {
		*results = append(*results, node.Node)
		return
	}

	raycastNode(node.Left, ray, mask, results)
	raycastNode(node.Right, ray, mask, results)
}

// mergeAABB merges two AABBs into one.
func mergeAABB(a, b AABB) AABB {
	return AABB{
		Min: mgl32.Vec3{
			min(a.Min.X(), b.Min.X()),
			min(a.Min.Y(), b.Min.Y()),
			min(a.Min.Z(), b.Min.Z()),
		},
		Max: mgl32.Vec3{
			max(a.Max.X(), b.Max.X()),
			max(a.Max.Y(), b.Max.Y()),
			max(a.Max.Z(), b.Max.Z()),
		},
	}
}

// volume calculates the volume of an AABB.
func volume(aabb AABB) float32 {
	size := aabb.Max.Sub(aabb.Min)
	return size.X() * size.Y() * size.Z()
}

// min returns the smaller of two float32 values.
func min(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

// max returns the larger of two float32 values.
func max(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}

// Scene represents the collection of objects and the acceleration structure.
type Scene struct {
	Tree DynamicAABBTree
}

// RaycastResult represents the result of a raycast query.
type RaycastResult struct {
	Node     *SceneNode // The intersected node
	Distance float32    // Distance along the ray
}

// Raycast performs a raycast against the scene and returns the closest hit, considering MaxLength.
func (s *Scene) Raycast(ray Ray, mask uint32) *RaycastResult {
	candidates := s.Tree.Raycast(ray, mask)
	var closest *RaycastResult
	for _, node := range candidates {
		// Perform narrowphase intersection test
		if hit, distance := node.Raycast(ray); hit {
			if closest == nil || distance < closest.Distance {
				closest = &RaycastResult{Node: node, Distance: distance}
			}
		}
	}
	return closest
}

// RaycastAny performs a raycast against the scene and returns any hit, without guaranteeing the closest.
// This method is optimized for scenarios where the first hit is sufficient, such as visibility checks.
func (s *Scene) RaycastAny(ray Ray, mask uint32) *RaycastResult {
	candidates := s.Tree.Raycast(ray, mask)
	for _, node := range candidates {
		// Perform narrowphase intersection test
		if hit, distance := node.Raycast(ray); hit {
			return &RaycastResult{Node: node, Distance: distance}
		}
	}
	return nil
}

// OverlapQuery performs an AABB overlap query against the scene.
func (s *Scene) OverlapQuery(aabb AABB, mask uint32) []SceneNode {
	candidates := s.Tree.Query(aabb, mask)
	var results []SceneNode
	for _, node := range candidates {
		// Perform narrowphase AABB-AABB intersection test
		if node.IntersectAABB(aabb) {
			results = append(results, *node)
		}
	}
	return results
}

// intersectAABB checks if two AABBs overlap.
func intersectAABB(a, b AABB) bool {
	return (a.Min.X() <= b.Max.X() && a.Max.X() >= b.Min.X()) &&
		(a.Min.Y() <= b.Max.Y() && a.Max.Y() >= b.Min.Y()) &&
		(a.Min.Z() <= b.Max.Z() && a.Max.Z() >= b.Min.Z())
}

// PointContainmentQuery checks which nodes contain a given point.
func (s *Scene) PointContainmentQuery(point mgl32.Vec3, mask uint32) []SceneNode {
	candidates := s.Tree.Query(AABB{Min: point, Max: point}, mask)
	var results []SceneNode
	for _, node := range candidates {
		// Perform narrowphase point containment test
		if containsPoint(node.Bounds, point) {
			results = append(results, *node)
		}
	}
	return results
}

// containsPoint checks if a point is inside an AABB.
func containsPoint(aabb AABB, point mgl32.Vec3) bool {
	return (point.X() >= aabb.Min.X() && point.X() <= aabb.Max.X()) &&
		(point.Y() >= aabb.Min.Y() && point.Y() <= aabb.Max.Y()) &&
		(point.Z() >= aabb.Min.Z() && point.Z() <= aabb.Max.Z())
}

// intersectRayAABB checks if a ray intersects an AABB and returns the hit status and distance.
func intersectRayAABB(ray Ray, aabb AABB) (bool, float32) {
	tMin := (aabb.Min.X() - ray.Origin.X()) / ray.Direction.X()
	tMax := (aabb.Max.X() - ray.Origin.X()) / ray.Direction.X()

	if tMin > tMax {
		tMin, tMax = tMax, tMin
	}

	tyMin := (aabb.Min.Y() - ray.Origin.Y()) / ray.Direction.Y()
	tyMax := (aabb.Max.Y() - ray.Origin.Y()) / ray.Direction.Y()

	if tyMin > tyMax {
		tyMin, tyMax = tyMax, tyMin
	}

	if tMin > tyMax || tyMin > tMax {
		return false, 0
	}

	if tyMin > tMin {
		tMin = tyMin
	}
	if tyMax < tMax {
		tMax = tyMax
	}

	tzMin := (aabb.Min.Z() - ray.Origin.Z()) / ray.Direction.Z()
	tzMax := (aabb.Max.Z() - ray.Origin.Z()) / ray.Direction.Z()

	if tzMin > tzMax {
		tzMin, tzMax = tzMax, tzMin
	}

	if tMin > tzMax || tzMin > tMax {
		return false, 0
	}

	if tzMin > tMin {
		tMin = tzMin
	}
	if tzMax < tMax {
		tMax = tzMax
	}

	if tMin < 0 && tMax < 0 {
		return false, 0
	}

	distance := tMin
	if tMin < 0 {
		distance = tMax
	}

	return true, distance
}
