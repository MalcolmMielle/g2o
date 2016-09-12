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

#include "edge_link_xy.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {

  EdgeLinkXY_malcolm::EdgeLinkXY_malcolm() :
    EdgeSE2PointXY()
  {
  }

 
#ifdef G2O_HAVE_OPENGL
  EdgeLinkXY_malcolmDrawAction::EdgeLinkXY_malcolmDrawAction(): DrawAction(typeid(EdgeLinkXY_malcolm).name()){}

  HyperGraphElementAction* EdgeLinkXY_malcolmDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                HyperGraphElementAction::Parameters*  params_){
    if (typeid(*element).name()!=_typeName)
      return 0;

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;


    EdgeLinkXY_malcolm* e =  static_cast<EdgeLinkXY_malcolm*>(element);
    VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
    VertexPointXY* toEdge   = static_cast<VertexPointXY*>(e->vertex(1));
    if (! fromEdge)
      return this;
    Vector2D p=e->measurement();
    glPushAttrib(GL_ENABLE_BIT|GL_LIGHTING|GL_COLOR);
    glDisable(GL_LIGHTING);
    if (!toEdge){
      p=fromEdge->estimate()*p;
      glColor3f(POSE_EDGE_LINK_XY_GHOST_COLOR);
      glPushAttrib(GL_POINT_SIZE);
      glPointSize(3);
      glBegin(GL_POINTS);
      glVertex3f((float)p.x(),(float)p.y(),0.f);
      glEnd();
      glPopAttrib();
    } else {
      p=toEdge->estimate();
      glColor3f(POSE_EDGE_LINK_XY_COLOR);
    }
    glBegin(GL_LINES);
    glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),0.f);
    glVertex3f((float)p.x(),(float)p.y(),0.f);
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

} // end namespace
