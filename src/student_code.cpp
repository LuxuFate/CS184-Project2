#include "student_code.h"
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

    Vector2D lerp(Vector2D pointa, Vector2D pointb, double t) {
        Vector2D intermediate_point = (1 - t) * pointa + (t * pointb);
        return intermediate_point;
    }

    Vector3D lerp(Vector3D pointa, Vector3D pointb, double t) {
        Vector3D intermediate_point = (1 - t) * pointa + (t * pointb);
        return intermediate_point;
    }

  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
      vector<Vector2D> intermediate_points = vector<Vector2D>();
      for (int i = 0; i < points.size()-1; i++) {
          intermediate_points.push_back(lerp(points[i], points[i+1], t));
      }
      return intermediate_points;
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
      vector<Vector3D> intermediate_points = vector<Vector3D>();
      for (int i = 0; i < points.size()-1; i++) {
          intermediate_points.push_back(lerp(points[i], points[i+1], t));
      }
      return intermediate_points;
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
      if (points.size() == 1){
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
      vector<Vector3D> pointstemp;
      for (int i = 0; i < controlPoints.size(); i++){
          pointstemp.push_back(evaluate1D(controlPoints[i], u));

      }
      return evaluate1D(pointstemp, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
      HalfedgeCIter h = halfedge(); //get the first half-edge of the face
      h = h->twin();
      Vector3D p0 = h->vertex()->position; //og
      HalfedgeCIter original = h;
      h = h->next()->twin();
      Vector3D vec = Vector3D(0, 0, 0);
      while(h != original) {
          Vector3D p1 = h->vertex()->position;
          h = h->next()->twin();
          Vector3D p2 = h->vertex()->position;
          vec += cross(p0 - p1, p2 - p0);
      }
      return vec.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();
      
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();
      
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();
      
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();
      
      if (f0->isBoundary() || f1->isBoundary()) {
          return e0;
      }
      
      //next, twin, vertex, edge, face
      h0->setNeighbors(h1, h3, v3, e0, f0);
      h1->setNeighbors(h2, h7, v2, e2, f0);
      h2->setNeighbors(h0, h8, v0, e3, f0);
      h3->setNeighbors(h4, h0, v2, e0, f1);
      h4->setNeighbors(h5, h9, v3, e4, f1);
      h5->setNeighbors(h3, h6, v1, e1, f1);
      h6->setNeighbors(h6->next(), h5, v2, e1, h6->face());
      h7->setNeighbors(h7->next(), h1, v0, e2, h7->face());
      h8->setNeighbors(h8->next(), h2, v3, e3, h8->face());
      h9->setNeighbors(h9->next(), h4, v1, e4, h9->face());
      
      v0->halfedge() = h2;
      v1->halfedge() = h5;
      v2->halfedge() = h3;
      v3->halfedge() = h0;
      
      e0->halfedge() = h0;
      e1->halfedge() = h5;
      e2->halfedge() = h1;
      e3->halfedge() = h2;
      e4->halfedge() = h4;
      
      f0->halfedge() = h0;
      f1->halfedge() = h3;
      
      return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();
      
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();
      
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();
      
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();
      
      if (f0->isBoundary() || f1->isBoundary()) {
          return e0->halfedge()->vertex();
      }
      
      VertexIter v4 = newVertex();
      v4->position = (v0->position + v1->position)/2;
      
      EdgeIter e5 = newEdge();
      EdgeIter e6 = newEdge();
      EdgeIter e7 = newEdge();
      
      FaceIter f2 = newFace();
      FaceIter f3 = newFace();
      
      HalfedgeIter h10 = newHalfedge();
      HalfedgeIter h11 = newHalfedge();
      HalfedgeIter h12 = newHalfedge();
      HalfedgeIter h13 = newHalfedge();
      HalfedgeIter h14 = newHalfedge();
      HalfedgeIter h15 = newHalfedge();
      
      //next, twin, vertex, edge, face
      h0->setNeighbors(h1, h3, v4, e0, f0);
      h1->setNeighbors(h10, h6, v1, e1, f0);
      h2->setNeighbors(h14, h7, v2, e2, f3);
      h3->setNeighbors(h11, h0, v1, e0, f1);
      h4->setNeighbors(h12, h8, v0, e3, f2);
      h5->setNeighbors(h3, h9, v3, e4, f1);
      h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());
      h7->setNeighbors(h7->next(), h2, v0, e2, h7->face());
      h8->setNeighbors(h8->next(), h4, v3, e3, h8->face());
      h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());
      h10->setNeighbors(h0, h15, v2, e7, f0);
      h11->setNeighbors(h5, h12, v4, e6, f1);
      h12->setNeighbors(h13, h11, v3, e6, f2);
      h13->setNeighbors(h4, h14, v4, e5, f2);
      h14->setNeighbors(h15, h13, v0, e5, f3);
      h15->setNeighbors(h2, h10, v4, e7, f3);
    
      v0->halfedge() = h4;
      v1->halfedge() = h1;
      v2->halfedge() = h2;
      v3->halfedge() = h5;
      v4->halfedge() = h13;
      
      e0->halfedge() = h0;
      e1->halfedge() = h1;
      e2->halfedge() = h2;
      e3->halfedge() = h4;
      e4->halfedge() = h5;
      e5->halfedge() = h13;
      e6->halfedge() = h11;
      e7->halfedge() = h15;
      
      f0->halfedge() = h0;
      f1->halfedge() = h3;
      f2->halfedge() = h13;
      f3->halfedge() = h14;
      
      return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

  // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
      // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
      // a vertex of the original mesh.
      VertexIter v = mesh.verticesBegin();
      while (v != mesh.verticesEnd()){
          double n = v->degree();
          double u = 3.0/(8 * n);
          if (n == 3) {
              u = 3.0/16;
          }
          VertexIter next = v;
          next++;
          Vector3D onps;
          HalfedgeCIter original = v -> halfedge();
          HalfedgeCIter hedge = original->twin();
          do {
            onps += hedge->vertex()->position;
            hedge = hedge->next()->twin();
          } while(hedge != original->twin());
          Vector3D centroid = ((1 - n*u) * v -> position) + u * onps;
          v -> newPosition = centroid;
          v -> isNew = false;
          v = next;
      }
        
      // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
      EdgeIter e = mesh.edgesBegin();
      int counter = 0;
      while(e != mesh.edgesEnd()){
          EdgeIter next = e;
          next++;
          Vector3D A = e->halfedge()->vertex()->position;
          Vector3D B = e->halfedge()->twin()->vertex()->position;
          Vector3D C = e->halfedge()->next()->next()->vertex()->position;
          Vector3D D = e->halfedge()->twin()->next()->next()->vertex()->position;
          e -> newPosition = (3.0/8)*(A+B)+(1.0/8)*(C+D);
          e -> isNew = true;
          e = next;
          counter += 1;
      }
      // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some information about which subdivide edges come from splitting an edge in the original mesh, and which edges are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
      EdgeIter e1 = mesh.edgesBegin();
      while(counter > 0){
              EdgeIter next = e1;
              next++;
              VertexIter v4 = mesh.splitEdge(e1);
                  
              HalfedgeIter h13 = v4->halfedge();
              h13->edge()->isNew = false;
              HalfedgeIter h15 = h13->twin()->next();
              h15->edge()->isNew = true;
              HalfedgeIter h0 = h15->twin()->next();
              h0->edge()->isNew = false;
              HalfedgeIter h11 = h0->twin()->next();
              h11->edge()->isNew = true;
              v4->newPosition = e1->newPosition;
              v4->isNew = true;
              e1 = next;
              counter -= 1;
      }
      // 4. Flip any new edge that connects an old and new vertex.
      EdgeIter e2 = mesh.edgesBegin();
      while(e2 != mesh.edgesEnd()){
          EdgeIter next = e2;
          next++;
          if (e2->isNew) {
              HalfedgeIter h = e2->halfedge();
              if ((h->vertex()->isNew && !h->twin()->vertex()->isNew)||(!h->vertex()->isNew && h->twin()->vertex()->isNew)) {
                  mesh.flipEdge(e2);
              }
          }
          e2 = next;
      }
      // 5. Copy the new vertex positions into final Vertex::position.
      VertexIter v1 = mesh.verticesBegin();
      while (v1 != mesh.verticesEnd()){
          VertexIter next = v1;
          next++;
          v1->position = v1->newPosition;
          v1 = next;
      }
          
          
  }
}
