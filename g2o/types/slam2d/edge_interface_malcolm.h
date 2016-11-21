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

#ifndef G2O_EDGE_MALCOLM_INETRFACE_H
#define G2O_EDGE_MALCOLM_INETRFACE_H

#include "vertex_se2.h"
#include "edge_se2.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o_types_slam2d_api.h"

namespace g2o {

  /**
   * \brief Custom interface for easily changing all my nodes
   */
  class G2O_TYPES_SLAM2D_API EdgeInterfaceMalcolm
  {
    public:
	  g2o::SE2 _malcolm_original_value;
	  double _malcolm_age;
	  
//       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeInterfaceMalcolm() : _malcolm_age(1){};
	  
	  virtual g2o::SE2 getOriginalValue(){return _malcolm_original_value;}
	  virtual void setOriginalValue(const g2o::SE2& orig_val){_malcolm_original_value = orig_val;}
	  virtual double getAge(){return _malcolm_age;}
	  virtual void setAge(double a){_malcolm_age = a;}
	  

  };



} // end namespace

#endif

