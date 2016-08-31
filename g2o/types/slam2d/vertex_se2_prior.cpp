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

#include "vertex_se2_prior.h"
// #include <typeinfo>

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {

  VertexSE2Prior::VertexSE2Prior() :
    VertexSE2()
  {
  }
  
//    bool VertexSE2Prior::read(std::istream& is)
//   {
//     Vector3D p;
//     is >> p[0] >> p[1] >> p[2];
//     setEstimate(p);
//     return true;
//   }
// 
//   bool VertexSE2Prior::write(std::ostream& os) const
//   {
// 	  std::cout << "WRITE PRIOR" << std::endl; 
//     Vector3D p = estimate().toVector();
//     os << p[0] << " " << p[1] << " " << p[2];
//     return os.good();
//   }


#ifdef G2O_HAVE_OPENGL
//   VertexSE2PriorDrawAction::VertexSE2PriorDrawAction(): DrawAction(typeid(VertexSE2Prior).name()){
//     _drawActions = 0;
//   }
// 
//   bool VertexSE2PriorDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
//     if (!DrawAction::refreshPropertyPtrs(params_))
//       return false;
//     if (_previousParams){
//       _triangleX = _previousParams->makeProperty<FloatProperty>(_typeName + "::TRIANGLE_X", .2f);
//       _triangleY = _previousParams->makeProperty<FloatProperty>(_typeName + "::TRIANGLE_Y", .05f);
//     } else {
//       _triangleX = 0;
//       _triangleY = 0;
//     }
//     return true;
//   }
// 
// 
//   HyperGraphElementAction* VertexSE2PriorDrawAction::operator()(HyperGraph::HyperGraphElement* element,
//                  HyperGraphElementAction::Parameters* params_){
//    if (typeid(*element).name()!=_typeName)
//       return 0;
//     initializeDrawActionsCache();
//     refreshPropertyPtrs(params_);
// 
//     if (! _previousParams)
//       return this;
// 
//     if (_show && !_show->value())
//       return this;
// 
//     VertexSE2Prior* that = static_cast<VertexSE2Prior*>(element);
// 
//     glColor3f(POSE_VERTEX_PRIOR_COLOR);
//     glPushMatrix();
//     glTranslatef((float)that->estimate().translation().x(),(float)that->estimate().translation().y(),0.f);
//     glRotatef((float)RAD2DEG(that->estimate().rotation().angle()),0.f,0.f,1.f);
//     opengl::drawArrow2D((float)_triangleX->value(), (float)_triangleY->value(), (float)_triangleX->value()*.3f);
//     drawCache(that->cacheContainer(), params_);
//     drawUserData(that->userData(), params_);
//     glPopMatrix();
//     return this;
//   }
#endif


} // end namespace
