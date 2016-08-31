// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_VERTEX_SE3_PRIOR_
#define G2O_VERTEX_SE3_PRIOR_

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "isometry3d_mappings.h"
#include "g2o_types_slam3d_api.h"
#include "vertex_se3.h"

namespace g2o {

/**
 * \brief 3D pose Vertex, represented as an Isometry3D
 *
 * 3D pose vertex, represented as an Isometry3D, i.e., an affine transformation
 * which is constructed by only concatenating rotation and translation
 * matrices. Hence, no scaling or projection.  To avoid that the rotational
 * part of the Isometry3D gets numerically unstable we compute the nearest
 * orthogonal matrix after a large number of calls to the oplus method.
 * 
 * The parameterization for the increments constructed is a 6d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 */
  class G2O_TYPES_SLAM3D_API VertexSE3Prior : public VertexSE3
  {
    public:
      VertexSE3Prior();
  };

  /**
   * \brief write the vertex to some Gnuplot data file
   */
//   class VertexSE3PriorWriteGnuplotAction: public WriteGnuplotAction {
//     public:
//       VertexSE3WriteGnuplotAction();
//       virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
//           HyperGraphElementAction::Parameters* params_ );
//   };

#ifdef G2O_HAVE_OPENGL
  /**
   * \brief visualize the 3D pose vertex
   */
  class G2O_TYPES_SLAM3D_API VertexSE3PriorDrawAction: public DrawAction{
    public:
      VertexSE3PriorDrawAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_);
    protected:
      virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
      FloatProperty* _triangleX, *_triangleY;
  };
#endif

} // end namespace

#endif
