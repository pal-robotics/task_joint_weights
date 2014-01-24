from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.task_joint_weights import TaskJointWeights
from dynamic_graph import plug
from dynamic_graph.sot.core import *

class MetaTaskJointWeights(object):
    name=''
    dyn=0
    robot=0
    task=0       
    def createTask(self,selec,diag,gain,sampleInterval):
        self.task = TaskJointWeights(self.name)
        self.task.setSize(self.robot.dimension)
        self.task.selec.value = selec
        if(diag):
            self.task.setWeights(diag)
        else:
            self.dyn.signal('inertia').recompute(0)
            plug(self.dyn.signal('inertia'),self.task.signal('weights'))
        self.task.controlGain.value = gain
        self.task.setSampleInterval(sampleInterval)
    def __init__(self,name,selec,robot,diag=None,gain=1000,sampleInterval=0):
        self.name = name
        self.dyn = robot.dynamic
        self.robot = robot
        self.createTask(selec,diag,gain,sampleInterval)

