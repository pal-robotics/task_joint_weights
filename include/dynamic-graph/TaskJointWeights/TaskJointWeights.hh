/*
 * Copyright 2011, Nicolas Mansard, LAAS-CNRS
 *
 * This file is part of sot-dyninv.
 * sot-dyninv is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-dyninv is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-dyninv.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __sot_TaskJointWeights_H__
#define __sot_TaskJointWeights_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <sot/core/task.hh>
#include <sot/core/flags.hh>

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class TaskJointWeights
        :public TaskAbstract
        ,public EntityHelper<TaskJointWeights>
{

public: /* --- CONSTRUCTOR ---- */

    TaskJointWeights( const std::string& name );

public: /* --- ENTITY INHERITANCE --- */

    DYNAMIC_GRAPH_ENTITY_DECL();
    virtual void display( std::ostream& os ) const;

public:  /* --- SIGNALS --- */

    DECLARE_SIGNAL_IN(velocity,ml::Vector);
    DECLARE_SIGNAL_IN(dt,double);
    DECLARE_SIGNAL_IN(controlGain,double);
    DECLARE_SIGNAL_IN(weights,ml::Matrix);
    DECLARE_SIGNAL_OUT(activeSize,int);

public:  /* --- COMPUTATION --- */
    dg::sot::VectorMultiBound& computeTask( dg::sot::VectorMultiBound& res,int time );
    ml::Matrix& computeJacobian( ml::Matrix& J,int time );
    void setWeights(const ml::Vector& weightsIn);

}; // class TaskJointWeights

} // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __sot_TaskJointWeights_H__
