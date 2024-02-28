#include "student_code.h"
#include "halfEdgeMesh.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    //interate through points and lerp between them
    std::vector<Vector2D> newPoints;
    //base case
    if (points.size() == 1) {
      return points;
    }
    for (int i = 0; i < points.size() - 1; i++) {
      newPoints.push_back((1 - t) * points[i] + t * points[i + 1]);
    }
    return newPoints;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    //same for 3D
    std::vector<Vector3D> newPoints;
    if (points.size() == 1) {
      return points;
    }
    for (int i = 0; i < points.size() - 1; i++) {
      newPoints.push_back((1 - t) * points[i] + t * points[i + 1]);
    }
    return newPoints;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    //base case
    if(points.size() == 1) {
      return points[0];
    }
    return evaluate1D(evaluateStep(points, t), t);
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    //get size of outer loop
    int n = controlPoints.size();

    //create vector of points
    std::vector<Vector3D> points;
    for (int i = 0; i < n; i++) {
      points.push_back(evaluate1D(controlPoints[i], u));
    }
    return evaluate1D(points, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D result = Vector3D(0, 0, 0);
    HalfedgeCIter h = halfedge();
    //iterate through all halfedges and get area and normal using points
    do {
      if (h->face()->isBoundary()) {
        h = h->twin()->next();
        continue;
      }
      Vector3D p0 = position;
      Vector3D p1 = h->next()->vertex()->position;
      Vector3D p2 = h->next()->next()->vertex()->position;
      Vector3D normal = cross(p1 - p0, p2 - p0);
      //weight it by area
      double area = normal.norm()/2;
      result += area * normal;
      h = h->twin()->next();
    } while (h != halfedge());
    return result.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    //get all half edges
    //get inner half edges
    HalfedgeIter i0 = e0->halfedge();
    HalfedgeIter i1 =i0->next();
    HalfedgeIter i2 = i1->next();
    HalfedgeIter i3 = i0->twin();
    HalfedgeIter i4 = i3->next();
    HalfedgeIter i5 = i4->next();

    HalfedgeIter o0 = i1->twin();
    HalfedgeIter o1 = i2->twin();
    HalfedgeIter o2 = i4->twin();
    HalfedgeIter o3 = i5->twin();
    //get edges
    EdgeIter e1 = i1->edge();
    EdgeIter e2 = i2->edge();
    EdgeIter e3 = i4->edge();
    EdgeIter e4 = i5->edge();

    //get vertices
    VertexIter b = i0->vertex();
    VertexIter a = i2->vertex();
    VertexIter c = i3->vertex();
    VertexIter d = i5->vertex();

    //get faces
    FaceIter f0 = i0->face();
    FaceIter f1 = i3->face();

    //check boundary before flipping
    if (e0->isBoundary()) {
      return e0;
    }

    //use setNeighbors
    i0->setNeighbors(i1, i3, a, e0, f0);
    i1->setNeighbors(i2, o3, d, e4, f0);
    i2->setNeighbors(i0, o0, c, e1, f0);
    i3->setNeighbors(i4, i0, d, e0, f1);
    i4->setNeighbors(i5, o1, a, e2, f1);
    i5->setNeighbors(i3, o2, b, e3, f1);
    //DONT CHANGE FACES AND NEXT
    o0->setNeighbors(o0->next(), i2, a, e1, o0->face());
    o1->setNeighbors(o1->next(), i4, b, e2, o1->face());
    o2->setNeighbors(o2->next(), i5, d, e3, o2->face());
    o3->setNeighbors(o3->next(), i1, c, e4, o3->face());
    
    //change vertex stuff
    b->halfedge() = i5;
    c->halfedge() = i2;
    a->halfedge() = i4;
    d->halfedge() = i1;

    //change edge stuff
    e0->halfedge() = i0;
    e1->halfedge() = i2;
    e2->halfedge() = i4;
    e3->halfedge() = i5;
    e4->halfedge() = i1;

    //change face stuff so that it points to the right halfedge
    f0->halfedge() = i0;
    f1->halfedge() = i3;

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
        //get all half edges


    //check boundary before flipping
    if (e0->isBoundary()) {
      return VertexIter();
    }

    //get inner half edges
    HalfedgeIter i0 = e0->halfedge();
    HalfedgeIter i1 =i0->next();
    HalfedgeIter i2 = i1->next();
    HalfedgeIter i3 = i0->twin();
    HalfedgeIter i4 = i3->next();
    HalfedgeIter i5 = i4->next();

    HalfedgeIter o0 = i1->twin();
    HalfedgeIter o1 = i2->twin();
    HalfedgeIter o2 = i4->twin();
    HalfedgeIter o3 = i5->twin();
    //get edges
    EdgeIter e1 = i1->edge();
    EdgeIter e2 = i2->edge();
    EdgeIter e3 = i4->edge();
    EdgeIter e4 = i5->edge();

    //get vertices
    VertexIter b = i0->vertex();
    VertexIter a = i2->vertex();
    VertexIter c = i3->vertex();
    VertexIter d = i5->vertex();

    //get faces
    FaceIter f0 = i0->face();
    FaceIter f1 = i3->face();
  //create
    VertexIter v = newVertex();
    HalfedgeIter n0 = newHalfedge();
    HalfedgeIter n1 = newHalfedge();
    HalfedgeIter n2 = newHalfedge();
    HalfedgeIter n3 = newHalfedge();
    HalfedgeIter n4 = newHalfedge();
    HalfedgeIter n5 = newHalfedge();
    EdgeIter e5 = newEdge();
    EdgeIter e6 = newEdge();
    EdgeIter e7 = newEdge();
    FaceIter f2 = newFace();
    FaceIter f3 = newFace();

    //change stuff
    i0->setNeighbors(i1, i3, v, e0, f0);
    i1->setNeighbors(i2, o0, c, e1, f0);
    i2->setNeighbors(i0, n1, a, e5, f0);
    i3->setNeighbors(i4, i0, c, e0, f1);
    i4->setNeighbors(i5, n5, v, e7, f1);
    i5->setNeighbors(i3, o3, d, e4, f1);
    //dont change next or face
    o0->setNeighbors(o0->next(), i1, a, e1, o0->face());
    o1->setNeighbors(o1->next(), n2, b, e2, o1->face());
    o2->setNeighbors(o2->next(), n4, d, e3, o2->face());
    o3->setNeighbors(o3->next(), i5, c, e4, o3->face());
    //change new halfedges
    n0->setNeighbors(n1, n3, b, e6, f2);
    n1->setNeighbors(n2, i2, v, e5, f2);
    n2->setNeighbors(n0, o1, a, e2, f2);
    n3->setNeighbors(n4, n0, v, e6, f3);
    n4->setNeighbors(n5, o2, b, e3, f3);
    n5->setNeighbors(n3, i4, d, e7, f3);
    //vertecies
    //get midpoint of original edge
    v->position = (b->position + c->position) * 0.5;
    v->isNew = true;
    
    a->halfedge() = n2;
    b->halfedge() = n0;
    c->halfedge() = i1;
    d->halfedge() = i5;
    v->halfedge() = i0;

    //edgess
    e0->halfedge() = i0;
    e1->halfedge() = i1;
    e2->halfedge() = n2;
    e3->halfedge() = n4;
    e4->halfedge() = i5;
    e5->halfedge() = i2;
    e6->halfedge() = n0;
    e7->halfedge() = i4;
    //0 6 two halfs of original edge 5 7 new edges
    e0->isNew = false;
    e5->isNew = true;
    e6->isNew = false;
    e7->isNew = true;

    //face
    f0->halfedge() = i0;
    f1->halfedge() = i3;
    f2->halfedge() = n0;
    f3->halfedge() = n3;

    return v;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

    //loop through all 
    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    //loop throuhg vertex
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      //is old
      v->isNew = 0;
      //get weighted average of surrounding old verticies
      Vector3D sum = Vector3D(0, 0, 0);
      HalfedgeIter half = v->halfedge();
      do{
        
        sum += half->twin()->vertex()->position;
        half = half->twin()->next();

      } while (half != v->halfedge());
      //get new position
      //n = vertex degree
      float n = v->degree();
      float u;
      if (n == 3.0) {
        u = 3.0/16.0;
      } else {
        u = 3.0/(8.0 * n);
      }
      v->newPosition = v->position * (1.0 - n * u) + sum * u;
    }
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      //set isNew to falseso we know that we need to split it 
      e->isNew = false;
 
      VertexCIter a = e->halfedge()->vertex();
      VertexCIter b = e->halfedge()->twin()->vertex();
      VertexCIter c = e->halfedge()->next()->next()->vertex();
      VertexCIter d = e->halfedge()->twin()->next()->next()->vertex();
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
      //set new position using 1/8 3/8 formula
      //3/8 * (A + B) + 1/8 * (C + D)
      e->newPosition = 3.0/8.0 * (a->position + b->position) + 1.0/8.0 * (c->position + d->position);
    }
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    for (EdgeIter e = mesh.edgesBegin(); e!= mesh.edgesEnd(); e++) {
      //get the two vertexes
      VertexIter v0 = e->halfedge()->vertex();
      VertexIter v1 = e->halfedge()->twin()->vertex();
      //check both vertexes
      if (!(v0->isNew || v1->isNew)) {
        //split the edge
        VertexIter v = mesh.splitEdge(e);
        //set new position
        v->newPosition = e->newPosition;
      }
    }
     
    // 4. Flip any new edge that connects an old and new vertex.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      //get the two vertexes
      VertexIter v0 = e->halfedge()->vertex();
      VertexIter v1 = e->halfedge()->twin()->vertex();
      //if one is new and one is old
      if ((v0->isNew != v1->isNew) && e->isNew) {
        //flip the edge
        mesh.flipEdge(e);
      }
    }
        // 5. Copy the new vertex positions into final Vertex::position.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->position = v->newPosition;
      v->isNew = false;
    }
  }
}
