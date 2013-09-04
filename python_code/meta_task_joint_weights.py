from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.task_joint_weights import TaskJointWeights
from dynamic_graph import plug

from dynamic_graph import plug
from dynamic_graph.sot.core import *

class MetaTasJointkWeights(object):
    name=''
    dyn=0
    robot=0
    task=0       
    def createTask(self,diag):
        self.task = TaskJointWeights(self.name)
        if(diag):
            self.task.setWeights(diag)
        else:
            self.dyn.signal('inertia').recompute(0)
            plug(self.dyn.signal('inertia'),self.task.signal('weights'))
        #plug(self.dyn.position,self.task.velocity)
        plug(self.robot.device.oldControl,self.task.velocity)
        self.task.selec.value = toFlags(range(0,self.robot.dimension))
        self.task.controlGain.value = 1000 * 1000
        self.task.dt.value = 0.001
    def __init__(self,name,robot,diag=None):
        self.name = name
        self.dyn = robot.dynamic
        self.robot = robot
        self.createTask(diag)
