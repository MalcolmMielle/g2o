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

#ifndef G2O_EDGE_SE2_PRIOR_H
#define G2O_EDGE_SE2_PRIOR_H

#include "vertex_se2.h"
#include "edge_se2.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o_types_slam2d_api.h"
#include "edge_interface_malcolm.h"

namespace g2o {

  /**
   * \brief Prior for a two D pose
   */
  class G2O_TYPES_SLAM2D_API EdgeSE2Prior_malcolm : public EdgeSE2
  {
    public:
		EdgeInterfaceMalcolm interface;
// 	  g2o::SE2 _original_value;
//       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE2Prior_malcolm();
	  
// 	  g2o::SE2 getOriginalValue(){return _original_value;}
// 	  void setOriginalValue(const g2o::SE2& orig_val){_original_value = orig_val;}
	  

  };

  
  #ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_API EdgeSE2Prior_malcolmDrawAction: public DrawAction{
  public:
    EdgeSE2Prior_malcolmDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  protected:
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    FloatProperty* _triangleX, *_triangleY;

  };
	#endif

} // end namespace

#endif

