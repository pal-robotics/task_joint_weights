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
    ,sample_interval_(0)
    ,counter_(-1)
    ,CONSTRUCT_SIGNAL_IN(selec,Flags)
    ,CONSTRUCT_SIGNAL_IN(dt,double)
    ,CONSTRUCT_SIGNAL_IN(positionDes,ml::Vector)
    ,CONSTRUCT_SIGNAL_IN(position,ml::Vector)
    ,CONSTRUCT_SIGNAL_IN(velocity,ml::Vector)
    ,CONSTRUCT_SIGNAL_IN(controlGain,double)
    ,CONSTRUCT_SIGNAL_IN(weights,ml::Matrix)
    ,CONSTRUCT_SIGNAL_OUT(activeSize,int,positionDesSIN)

{

    //dynamicgraph::sot::DebugTrace::openFile("/tmp/weightsTask.txt");

    taskSOUT.setFunction( boost::bind(&TaskJointWeights::computeTask,this,_1,_2) );
    jacobianSOUT.setFunction( boost::bind(&TaskJointWeights::computeJacobian,this,_1,_2) );

    jacobianSOUT.addDependency( weightsSIN );
    jacobianSOUT.addDependency( activeSizeSOUT );

    taskSOUT.clearDependencies();
    taskSOUT.addDependency( velocitySIN );
    taskSOUT.addDependency( dtSIN );
    taskSOUT.addDependency( controlGainSIN );
    taskSOUT.addDependency( activeSizeSOUT );
    taskSOUT.addDependency( positionSIN );
    taskSOUT.addDependency( positionDesSIN );

    controlGainSIN = 1.0;
    dtSIN = 0.01;
    selecSIN = true;

    signalRegistration( weightsSIN << activeSizeSOUT << controlGainSIN << velocitySIN << positionSIN << positionDesSIN << dtSIN << selecSIN);

    // Commands
    std::string docstring;

    // setWeights
    docstring =
            "\n"
            " Set the weights matrix using an input vector."
            "\n";
    addCommand(std::string("setWeights"),new command::Setter<TaskJointWeights, ml::Vector>(*this, &TaskJointWeights::setWeights, docstring));

    // setPositionDes
    docstring =
            "\n"
            " Set the desired position."
            "\n";
    addCommand(std::string("setPositionDes"),new command::Setter<TaskJointWeights, ml::Vector>(*this, &TaskJointWeights::setPositionDes, docstring));

    // setPosition
    docstring =
            "\n"
            " Set the position."
            "\n";
    addCommand(std::string("setPosition"),new command::Setter<TaskJointWeights, ml::Vector>(*this, &TaskJointWeights::setPosition, docstring));

    // setVelocity
    docstring =
            "\n"
            " Set the velocity."
            "\n";
    addCommand(std::string("setVelocity"),new command::Setter<TaskJointWeights, ml::Vector>(*this, &TaskJointWeights::setVelocity, docstring));

    // setSize
    docstring =
            "\n"
            " Set the input size."
            "\n";
    addCommand(std::string("setSize"),new command::Setter<TaskJointWeights, unsigned int>(*this, &TaskJointWeights::setSize, docstring));

    // setSampleInterval
    docstring =
            "\n"
            " Set the sample interval for the weights computation."
            "\n";
    addCommand(std::string("setSampleInterval"),new command::Setter<TaskJointWeights, int>(*this, &TaskJointWeights::setSampleInterval, docstring));

}

/* ---------------------------------------------------------------------- */
/* --- COMPUTATION ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

void TaskJointWeights::setWeights(const ml::Vector& weightsIn){

    const Flags & selec = selecSIN(selecSIN.getTime());
    const int activeSize = activeSizeSOUT(activeSizeSOUT.getTime());

    //std::cout<<"selec: "<<selec<<std::endl;
    //std::cout<<"activeSize: "<<activeSize<<std::endl;

    ml::Matrix mat;
    mat.resize(activeSize,size_);
    mat.setZero();

    // Filter the diagonal vector accordingly to selec
    int idx = 0;
    for(unsigned int i=0; i<size_; i++ )
        if( selec(i) ){
            mat(idx,i) = weightsIn(i);
            idx++;
        }
    jacobianSOUT.clearDependencies();
    jacobianSOUT.setConstant(mat);

    //std::cout<<"Constant J: "<<mat<<std::endl;

}

void TaskJointWeights::setPositionDes(const ml::Vector& positionDes){

    positionDesSIN.setConstant(positionDes);
}

void TaskJointWeights::setPosition(const ml::Vector& position){

    positionSIN.setConstant(position);
}

void TaskJointWeights::setVelocity(const ml::Vector& velocity){

    velocitySIN.setConstant(velocity);
}

void TaskJointWeights::setSize(const unsigned int& size){

    size_ = size;
}

void TaskJointWeights::setSampleInterval(const int& sample_inteval){

    sample_interval_ = sample_inteval;
}

int& TaskJointWeights::activeSizeSOUT_function(int& res, int time)
{
    const Flags & selec = selecSIN(time);
    res = 0;
    for(unsigned int i=0;i<size_;++i )
    {
        if(selec(i)) res++;
    }
    return res;
}

dg::sot::VectorMultiBound& TaskJointWeights::computeTask( dg::sot::VectorMultiBound& res,int time )
{

    const Flags & selec = selecSIN(time);
    const ml::Vector & v = velocitySIN(time);
    const ml::Vector & p = positionSIN(time);
    const ml::Vector & pd = positionDesSIN(time);

    //assert(v.size() == p.size() && v.size() == pd.size() && p.size() == pd.size());

    const double dt = dtSIN(time);
    const double K = controlGainSIN(time)/dt;
    const int activeSize = activeSizeSOUT(time);

    err_.resize(size_);

    // Compute the position error
    for(unsigned int i=0;i<size_;++i )
        err_(i) = pd(i) - p(i);

    sotDEBUG(35) << "velocity = " << v << std::endl;
    sotDEBUG(35) << "positionDes = " << pd << std::endl;
    sotDEBUG(35) << "position = " << p << std::endl;
    sotDEBUG(35) << "err = " << err_ << std::endl;

    //std::cout<<"pos: "<<p<<std::endl;
    //std::cout<<"posDes: "<<pd<<std::endl;
    //std::cout<<"vel: "<<v<<std::endl;
    //std::cout<<"err: "<<err_<<std::endl;

    res.resize(activeSize);
    int idx = 0;
    for(unsigned int i=0;i<size_;++i )
    {
        if( selec(i) ){
            res[idx] = K * (err_(i) - dt * v(i));
            idx++;
        }
    }

    //std::cout<<"res: "<<res<<std::endl;

    //getchar();

    sotDEBUG(15) << "taskW = "<< res << std::endl;

    return res;
}

ml::Matrix& TaskJointWeights::computeJacobian( ml::Matrix& J,int time )
{

    if(counter_ == sample_interval_ || counter_ == -1){
        last_weights_ = weightsSIN(time); // Compute the weights
        counter_ = 0;
    }
    else{
        counter_++; // Use the last computed weights
    }

    const Flags & selec = selecSIN(time);
    const int activeSize = activeSizeSOUT(time);

    J.resize(activeSize,size_);
    J.setZero();

    int idx = 0;
    for(unsigned int i=0;i<size_;++i )
        if( selec(i) ){
            J(idx,i) = std::sqrt(last_weights_(i,i));
            idx++;
        }

    //std::cout<<"J: "<<J<<std::endl;

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

