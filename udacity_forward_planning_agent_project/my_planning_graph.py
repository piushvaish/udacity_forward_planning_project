
from itertools import chain, combinations, product
from aimacode.planning import Action
from aimacode.utils import expr

from layers import BaseActionLayer, BaseLiteralLayer, makeNoOp, make_node


class ActionLayer(BaseActionLayer):

    def _inconsistent_effects(self, actionA, actionB):
        """ Return True if an effect of one action negates an effect of the other

        See Also
        --------
        layers.ActionNode
        """
        # TODO: implement this function
        return actionA.effects & set([~e for e in actionB.effects])
        #raise NotImplementedError


    def _interference(self, actionA, actionB):
        """ Return True if the effects of either action negate the preconditions of the other 
        
        See Also
        --------
        layers.ActionNode
        """
        # TODO: implement this function
        #Boolean values of whether actionA negates actionB, vice versa        
        return any(~effect == prediction for effect, prediction in
                   chain(product(actionA.effects, actionB.preconditions),
                         product(actionB.effects, actionA.preconditions)))
          #raise NotImplementedError

    def _competing_needs(self, actionA, actionB):
        """ Return True if any preconditions of the two actions are pairwise mutex in the parent layer
        
        See Also
        --------
        layers.ActionNode
        layers.BaseLayer.parent_layer
        """
        # TODO: implement this function
         
        for parent_a1 in self.parents[actionA]:
            for parent_a2 in self.parents[actionB]:
                if self.parent_layer.is_mutex(parent_a1,parent_a2):
                    return True 
        #raise NotImplementedError


class LiteralLayer(BaseLiteralLayer):

    def _inconsistent_support(self, literalA, literalB):
        """ Return True if all ways to achieve both literals are pairwise mutex in the parent layer

        See Also
        --------
        layers.BaseLayer.parent_layer
        """
        # TODO: implement this function     
        return all(self.parent_layer.is_mutex(actionA, actionB)
                   for actionA, actionB in product(self.parents[literalA], self.parents[literalB]))
        #raise NotImplementedError

    def _negation(self, literalA, literalB):
        """ Return True if two literals are negations of each other """
        # TODO: implement this function
        return literalA == ~literalB
        #raise NotImplementedError


class PlanningGraph:
    def __init__(self, problem, state, serialize=True, ignore_mutexes=False):
        """
        Parameters
        ----------
        problem : PlanningProblem
            An instance of the PlanningProblem class

        state : tuple(bool)
            An ordered sequence of True/False values indicating the literal value
            of the corresponding fluent in problem.state_map

        serialize : bool
            Flag indicating whether to serialize non-persistence actions. Actions
            should NOT be serialized for regression search (e.g., GraphPlan), and
            _should_ be serialized if the planning graph is being used to estimate
            a heuristic
        """
        self._serialize = serialize
        self._is_leveled = False
        self._ignore_mutexes = ignore_mutexes
        self.goal = set(problem.goal)

        # make no-op actions that persist every literal to the next layer
        no_ops = [make_node(n, no_op=True) for n in chain(*(makeNoOp(s) for s in problem.state_map))]
        self._actionNodes = no_ops + [make_node(a) for a in problem.actions_list]
        
        # initialize the planning graph by finding the literals that are in the
        # first layer and finding the actions they they should be connected to
        literals = [s if f else ~s for f, s in zip(state, problem.state_map)]
        layer = LiteralLayer(literals, ActionLayer(), self._ignore_mutexes)
        layer.update_mutexes()
        self.literal_layers = [layer]
        self.action_layers = []

    def h_levelsum(self):
        """ Calculate the level sum heuristic for the planning graph

        The level sum is the sum of the level costs of all the goal literals
        combined. The "level cost" to achieve any single goal literal is the
        level at which the literal first appears in the planning graph. Note
        that the level cost is **NOT** the minimum number of actions to
        achieve a single goal literal.
        
        For example, if Goal1 first appears in level 0 of the graph (i.e.,
        it is satisfied at the root of the planning graph) and Goal2 first
        appears in level 3, then the levelsum is 0 + 3 = 3.

        Hint: expand the graph one level at a time and accumulate the level
        cost of each goal.

        See Also
        --------
        Russell-Norvig 10.3.1 (3rd Edition)
        """
        # TODO: implement this function

        self.fill()
        level_sum = 0
        for clause in self.goal:
            level = 0
            for s_level in self.literal_layers:
                match = [node for node in s_level if clause in s_level]
                if match: break
                level += 1
            if level == len(self.literal_layers):
                return False
            level_sum += level
        
        return level_sum
        #raise NotImplementedError

    def h_maxlevel(self):
        """ Calculate the max level heuristic for the planning graph

        The max level is the largest level cost of any single goal fluent.
        The "level cost" to achieve any single goal literal is the level at
        which the literal first appears in the planning graph. Note that
        the level cost is **NOT** the minimum number of actions to achieve
        a single goal literal.

        For example, if Goal1 first appears in level 1 of the graph and
        Goal2 first appears in level 3, then the levelsum is max(1, 3) = 3.

        Hint: expand the graph one level at a time until all goals are met.

        See Also
        --------
        Russell-Norvig 10.3.1 (3rd Edition)

        Notes
        -----
        WARNING: you should expect long runtimes using this heuristic with A*
        """
        # TODO: implement maxlevel heuristic
        level_sum = []
        self.fill()
        # for each goal in the problem, determine the level cost, then add them together
        for goal in self.goal:
            levels_list = enumerate(self.literal_layers)
            for level_cost, level in levels_list:
                #if goal is found in the level, add the level cost to the sum
                if goal in level:
                    level_sum.append(level_cost)
                    break                        
        return max(level_sum)
        #raise NotImplementedError

    def h_setlevel(self):
        """ Calculate the set level heuristic for the planning graph

        The set level of a planning graph is the first level where all goals
        appear such that no pair of goal literals are mutex in the last
        layer of the planning graph.

        Hint: expand the graph one level at a time until you find the set level

        See Also
        --------
        Russell-Norvig 10.3.1 (3rd Edition)

        Notes
        -----
        WARNING: you should expect long runtimes using this heuristic on complex problems
        """
        # TODO: implement setlevel heuristic
        self.fill()        
        for level_cost,level in enumerate(self.literal_layers):
            #print(level.parents)
            all_goals_met = True
            for goal in self.goal:
                if not goal in level:
                    all_goals_met = False
            if not all_goals_met:
                continue                
            goals_are_mutex = False
            for goalA in self.goal:
                for goalB in self.goal:
                    if level.is_mutex(goalA,goalB):
                        goals_are_mutex = True
            if not goals_are_mutex:
                return level_cost
            #raise NotImplementedError
        ##Reference
        # https://ocw.mit.edu/courses/electrical-elevel = 0
        # http://www.grastien.net/ban/teaching/06-planning5.pdf
        # http://planning.cs.uiuc.edu/ch2.pdf
        # https://github.com/aimacode/aima-python/blob/master/planning.py
        # https://github.com/Morgan-Allen
        #https://github.com/lucko515/air-cargo-planning-ai/blob/master/my_planning_graph.py
        #http://www-users.cselabs.umn.edu/classes/Spring-2017/csci4511/mutex.html
        #http://www.cs.nott.ac.uk/~psznza/G52PAS/lecture12.pdf
        #https://github.com/udacity/artificial-intelligence/blob/master/Projects/2_Classical%20Planning/pseudocode/heuristics.md

    ##############################################################################
    #                     DO NOT MODIFY CODE BELOW THIS LINE                     #
    ##############################################################################

    def fill(self, maxlevels=-1):
        """ Extend the planning graph until it is leveled, or until a specified number of
        levels have been added

        Parameters
        ----------
        maxlevels : int
            The maximum number of levels to extend before breaking the loop. (Starting with
            a negative value will never interrupt the loop.)

        Notes
        -----
        YOU SHOULD NOT THIS FUNCTION TO COMPLETE THE PROJECT, BUT IT MAY BE USEFUL FOR TESTING
        """
        while not self._is_leveled:
            if maxlevels == 0: break
            self._extend()
       
            maxlevels -= 1
      
        return self

    def _extend(self):
        """ Extend the planning graph by adding both a new action layer and a new literal layer

        The new action layer contains all actions that could be taken given the positive AND
        negative literals in the leaf nodes of the parent literal level.

        The new literal layer contains all literals that could result from taking each possible
        action in the NEW action layer. 
        """
        if self._is_leveled: return
        
        parent_literals = self.literal_layers[-1]
        parent_actions = parent_literals.parent_layer
        action_layer = ActionLayer(parent_actions, parent_literals, self._serialize, self._ignore_mutexes)
        literal_layer = LiteralLayer(parent_literals, action_layer, self._ignore_mutexes)

        for action in self._actionNodes:
            # actions in the parent layer are skipped because are added monotonically to planning graphs,
            # which is performed automatically in the ActionLayer and LiteralLayer constructors
            if action not in parent_actions and action.preconditions <= parent_literals:
                action_layer.add(action)
                literal_layer |= action.effects

                # add two-way edges in the graph connecting the parent layer with the new action
                parent_literals.add_outbound_edges(action, action.preconditions)
                action_layer.add_inbound_edges(action, action.preconditions)

                # # add two-way edges in the graph connecting the new literaly layer with the new action
                action_layer.add_outbound_edges(action, action.effects)
                literal_layer.add_inbound_edges(action, action.effects)

        action_layer.update_mutexes()
        literal_layer.update_mutexes()
        self.action_layers.append(action_layer)
        self.literal_layers.append(literal_layer)
        self._is_leveled = literal_layer == action_layer.parent_layer
