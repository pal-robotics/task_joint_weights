/*
 * Copyright (c) 2013, PAL Robotics, S.L.
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

/** \author Gennaro Raiola
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

//#define VP_DEBUG
#define VP_DEBUG_MODE 15
#include <sot/core/debug.hh>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/TaskJointWeights/TaskJointWeights.hh>


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph
{
namespace sot
{

namespace dg = ::dynamicgraph;


/* --- DG FACTORY ------------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskJointWeights,"TaskJointWeights");

/* ---------------------------------------------------------------------- */
/* --- CONSTRUCTION ----------------------------------------------------- */
/* ---------------------------------------------------------------------- */

TaskJointWeights::TaskJointWeights( const std::string & name )
    : TaskAbstract(name)
    ,CONSTRUCT_SIGNAL_IN(velocity,ml::Vector)
    ,CONSTRUCT_SIGNAL_IN(dt,double)
    ,CONSTRUCT_SIGNAL_IN(controlGain,double)
    ,CONSTRUCT_SIGNAL_IN(weights,ml::Matrix)
    ,CONSTRUCT_SIGNAL_OUT(activeSize,int,velocitySIN)
{

    //dynamicgraph::sot::DebugTrace::openFile("/tmp/weightsTask.txt");

    taskSOUT.setFunction( boost::bind(&TaskJointWeights::computeTask,this,_1,_2) );
    jacobianSOUT.setFunction( boost::bind(&TaskJointWeights::computeJacobian,this,_1,_2) );

    jacobianSOUT.addDependency( weightsSIN );

    taskSOUT.clearDependencies();
    taskSOUT.addDependency( dtSIN );
    taskSOUT.addDependency( velocitySIN );
    controlGainSIN = 1.0;
    signalRegistration( dtSIN << weightsSIN << activeSizeSOUT << controlGainSIN << velocitySIN );

    // Commands
    std::string docstring;

    // setWeights
    docstring =
            "\n"
            " Set the weights matrix using an input vector."
            "\n";
    addCommand(std::string("setWeights"),new command::Setter<TaskJointWeights, ml::Vector>(*this, &TaskJointWeights::setWeights, docstring));


}

/* ---------------------------------------------------------------------- */
/* --- COMPUTATION ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

void TaskJointWeights::setWeights(const ml::Vector& weightsIn){

    ml::Matrix mat;
    mat.setDiagonal(weightsIn);
    jacobianSOUT.clearDependencies();
    jacobianSOUT.setConstant(mat);
}

int& TaskJointWeights::activeSizeSOUT_function(int& res, int time)
{
    const int size = velocitySIN(time).size();
    res=0;
    for( int i=0;i<size;++i )
    {
        res++;
    }
    return res;
}

dg::sot::VectorMultiBound& TaskJointWeights::computeTask( dg::sot::VectorMultiBound& res,int time )
{
    const ml::Vector & velocity = velocitySIN(time);
    const double K = 1.0/(dtSIN(time)*controlGainSIN(time));
    const int size = velocity.size(), activeSize=activeSizeSOUT(time);
    sotDEBUG(35) << "velocity = " << velocity << std::endl;

    res.resize(activeSize); int idx=0;
    for( int i=0;i<size;++i )
    {
            res[idx++] = - K * velocity(i);
    }

    sotDEBUG(15) << "taskW = "<< res << std::endl;

    return res;
}

ml::Matrix& TaskJointWeights::computeJacobian( ml::Matrix& J,int time )
{
    const ml::Matrix & weights = weightsSIN(time);
    const int size = velocitySIN(time).size(), activeSize = activeSizeSOUT(time);
    J.resize(activeSize,size);	J.setZero();

    int idx=0;
    for( int i=0;i<size;++i )
    {
        if(i<6)
            J(idx,i) = 0.0;
        else
            J(idx,i) = std::sqrt(weights(idx,i));
        idx++;
    }

    return J;
}

/* ------------------------------------------------------------------------ */
/* --- DISPLAY ENTITY ----------------------------------------------------- */
/* ------------------------------------------------------------------------ */

void TaskJointWeights::
display( std::ostream& os ) const
{
    os << "TaskJointWeights " << name << ": " << std::endl;
}

} // namespace sot
} // namespace dynamicgraph

