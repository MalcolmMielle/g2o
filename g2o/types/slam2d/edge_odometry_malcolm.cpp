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

#include "edge_odometry_malcolm.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {

EdgeOdometry_malcolm::EdgeOdometry_malcolm() : EdgeSE2()
{

}


 
#ifdef G2O_HAVE_OPENGL
  EdgeOdometry_malcolmDrawAction::EdgeOdometry_malcolmDrawAction(): DrawAction(typeid(EdgeOdometry_malcolm).name()){}

  bool EdgeOdometry_malcolmDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (!DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _triangleX = _previousParams->makeProperty<FloatProperty>(_typeName + "::GHOST_TRIANGLE_X", .2f);
      _triangleY = _previousParams->makeProperty<FloatProperty>(_typeName + "::GHOST_TRIANGLE_Y", .05f);
    } else {
      _triangleX = 0;
      _triangleY = 0;
    }
    return true;
  }

  HyperGraphElementAction* EdgeOdometry_malcolmDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
               HyperGraphElementAction::Parameters* params_){
//     std::cout << "DRAW BASE" << std::endl;exit(0);
    if (typeid(*element).name()!=_typeName){
		std::cout << "Wrong name :(" <<std::endl;;
      return 0;
	}

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;

    EdgeOdometry_malcolm* e =  static_cast<EdgeOdometry_malcolm*>(element);
    VertexSE2* from = static_cast<VertexSE2*>(e->vertex(0));
    VertexSE2* to   = static_cast<VertexSE2*>(e->vertex(1));
    if (! from && ! to)
      return this;
    SE2 fromTransform;
    SE2 toTransform;
    glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING | GL_COLOR);
    glDisable(GL_LIGHTING);
    if (! from) {
      glColor3f(POSE_EDGE_GHOST_COLOR);
      toTransform = to->estimate();
      fromTransform = to->estimate()*e->measurement().inverse();
      // DRAW THE FROM EDGE AS AN ARROW
      glPushMatrix();
      glTranslatef((float)fromTransform.translation().x(), (float)fromTransform.translation().y(),0.f);
      glRotatef((float)RAD2DEG(fromTransform.rotation().angle()),0.f,0.f,1.f);
      opengl::drawArrow2D((float)_triangleX->value(), (float)_triangleY->value(), (float)_triangleX->value()*.3f);
      glPopMatrix();
    } else if (! to){
      glColor3f(POSE_EDGE_GHOST_COLOR);
      fromTransform = from->estimate();
      toTransform = from->estimate()*e->measurement();
      // DRAW THE TO EDGE AS AN ARROW
      glPushMatrix();
      glTranslatef(toTransform.translation().x(),toTransform.translation().y(),0.f);
      glRotatef((float)RAD2DEG(toTransform.rotation().angle()),0.f,0.f,1.f);
      opengl::drawArrow2D((float)_triangleX->value(), (float)_triangleY->value(), (float)_triangleX->value()*.3f);
      glPopMatrix();
    } else {
      glColor3f(POSE_EDGE_COLOR);
      fromTransform = from->estimate();
      toTransform = to->estimate();
    }
    glBegin(GL_LINES);
    glVertex3f((float)fromTransform.translation().x(),(float)fromTransform.translation().y(),0.f);
    glVertex3f((float)toTransform.translation().x(),(float)toTransform.translation().y(),0.f);
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

} // end namespace
