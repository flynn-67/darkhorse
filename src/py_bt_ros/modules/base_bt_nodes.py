from enum import Enum

# -----------------------------------------------------------------------------
# BT Node List Definition
# -----------------------------------------------------------------------------
class BTNodeList:
    CONTROL_NODES = [         
        'Sequence',
        'Fallback',
        'ReactiveSequence',
        'ReactiveFallback',
        'Parallel'
    ]

    ACTION_NODES = [
        # Custom actions will be added via main.py or other modules
    ]

    CONDITION_NODES = [
        'AlwaysFailure',
        'AlwaysSuccess',
        # Custom conditions will be added via main.py
    ]

    DECORATOR_NODES = [
    ]

# -----------------------------------------------------------------------------
# Status Enumeration
# -----------------------------------------------------------------------------
class Status(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

# -----------------------------------------------------------------------------
# Base Node Class
# -----------------------------------------------------------------------------
class Node:
    def __init__(self, name, **kwargs):
        """
        Base Node.
        **kwargs is added to handle extra attributes from XML (e.g., ID, x, y)
        without crashing.
        """
        self.name = name
        self.type = None
        self.status = None

    async def run(self, agent, blackboard):
        raise NotImplementedError
    
    def halt(self):
        pass

    def reset(self):
        self.status = None
        if hasattr(self, "children"):
            for child in self.children:
                child.reset()
    

# -----------------------------------------------------------------------------
# Control Nodes
# -----------------------------------------------------------------------------

# Sequence node: Runs child nodes in sequence until one fails
class Sequence(Node):
    def __init__(self, name, children, **kwargs):
        super().__init__(name, **kwargs)
        self.children = children
        self.current_child_index = 0  

    async def run(self, agent, blackboard):
        while self.current_child_index < len(self.children):
            status = await self.children[self.current_child_index].run(agent, blackboard)
            self.status = status

            if status == Status.RUNNING:
                return Status.RUNNING  
            elif status == Status.FAILURE:
                self.halt_children()
                self.current_child_index = 0  
                return Status.FAILURE
            elif status == Status.SUCCESS:
                self.current_child_index += 1  

        self.current_child_index = 0  
        self.halt_children()
        return Status.SUCCESS

    def halt_children(self):
        for child in self.children:
            child.halt() 

    def halt(self):
        self.current_child_index = 0


class ReactiveSequence(Node):
    def __init__(self, name, children, **kwargs):
        super().__init__(name, **kwargs)
        self.children = children

    async def run(self, agent, blackboard):
        for child in self.children:
            status = await child.run(agent, blackboard)
            self.status = status
            if status == Status.FAILURE:
                self.halt_children()
                return Status.FAILURE  
            if status == Status.RUNNING:
                return Status.RUNNING  
        self.halt_children()
        return Status.SUCCESS  

    def halt_children(self):
        for child in self.children:
            child.halt()  


# Fallback node: Runs child nodes in sequence until one succeeds
class Fallback(Node):
    def __init__(self, name, children, **kwargs):
        super().__init__(name, **kwargs)
        self.children = children
        self.current_child_index = 0  

    async def run(self, agent, blackboard):
        while self.current_child_index < len(self.children):
            status = await self.children[self.current_child_index].run(agent, blackboard)
            self.status = status

            if status == Status.RUNNING:
                return Status.RUNNING  
            elif status == Status.SUCCESS:
                self.halt_children()
                self.current_child_index = 0  
                return Status.SUCCESS
            elif status == Status.FAILURE:
                self.current_child_index += 1  

        self.current_child_index = 0  
        self.halt_children()
        return Status.FAILURE

    def halt_children(self):
        for child in self.children:
            child.halt()  

    def halt(self):
        self.current_child_index = 0            


class ReactiveFallback(Node):
    def __init__(self, name, children, **kwargs):
        super().__init__(name, **kwargs)
        self.children = children

    async def run(self, agent, blackboard):
        for child in self.children:
            status = await child.run(agent, blackboard)
            self.status = status
            if status == Status.SUCCESS:
                self.halt_children()
                return Status.SUCCESS  
            if status == Status.RUNNING:
                return Status.RUNNING  
        
        self.halt_children()
        return Status.FAILURE  

    def halt_children(self):
        for child in self.children:
            child.halt()  


# Parallel node: Modified to accept XML attributes safely
class Parallel(Node):
    def __init__(self, name, children, success_threshold=None, failure_threshold=None, **kwargs):
        """
        success_threshold (XML attribute): number of SUCCESS children required for overall SUCCESS
        failure_threshold (XML attribute): number of FAILURE children causing overall FAILURE
        """
        super().__init__(name, **kwargs)
        self.children = children
        
        # XML에서 'success_threshold'라는 이름으로 넘어오는 값을 처리
        if success_threshold is not None:
            self.success_count = int(success_threshold)
        else:
            self.success_count = len(children)

        # XML에서 'failure_threshold'라는 이름으로 넘어오는 값을 처리
        if failure_threshold is not None:
            self.failure_count = int(failure_threshold)
        else:
            self.failure_count = None  # None means ignore failures in final decision

    async def run(self, agent, blackboard):
        successes = 0
        failures = 0
        any_running = False

        # Tick all children sequentially within the same tick
        for child in self.children:
            status = await child.run(agent, blackboard)

            if status == Status.SUCCESS:
                successes += 1
            elif status == Status.FAILURE:
                failures += 1
            elif status == Status.RUNNING:
                any_running = True

        # Final decision after evaluating all children
        if successes >= self.success_count:
            self.halt_children()
            self.status = Status.SUCCESS  
            return self.status

        if self.failure_count is not None and failures >= self.failure_count:
            self.halt_children()
            self.status = Status.FAILURE  
            return self.status

        if any_running:
            self.status = Status.RUNNING  
            return self.status

        # All finished, thresholds not satisfied → FAILURE
        self.halt_children()
        self.status = Status.FAILURE  
        return self.status

    def halt_children(self):
        for child in self.children:
            child.halt()

    def halt(self):
        self.halt_children()


# -----------------------------------------------------------------------------
# Action / Condition Nodes
# -----------------------------------------------------------------------------

# Synchronous action node
class SyncAction(Node):
    def __init__(self, name, action, **kwargs):
        super().__init__(name, **kwargs)
        self.action = action
        self.type = "Action"

    async def run(self, agent, blackboard):
        result = self.action(agent, blackboard)
        blackboard[self.name] = result
        self.status = result
        return result

class SyncCondition(Node):
    def __init__(self, name, condition, **kwargs):
        super().__init__(name, **kwargs)
        self.condition = condition
        self.is_expanded = False
        self.type = "Condition"

    async def run(self, agent, blackboard):
        result = self.condition(agent, blackboard)
        blackboard[self.name] = {'status': result, 'is_expanded': self.is_expanded} 
        self.status = result
        return result

    def set_expanded(self):
        self.is_expanded = True


# ---- Helper: AlwaysFailure & AlwaysSuccess -----------------------
class AlwaysFailure(SyncCondition):
    def __init__(self, name, agent, **kwargs):
        # agent 인자는 여기서 쓰지 않지만 시그니처 호환성을 위해 유지
        super().__init__(name, self._check, **kwargs)

    def _check(self, agent, blackboard):
        return Status.FAILURE

class AlwaysSuccess(SyncCondition):
    def __init__(self, name, agent, **kwargs):
        super().__init__(name, self._check, **kwargs)

    def _check(self, agent, blackboard):
        return Status.SUCCESS