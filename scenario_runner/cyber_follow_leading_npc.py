#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Cyber Attack : Follow leading vehicle scenario

The scenario realizes a common driving behavior, in which the
user-controlled ego vehicle follows a leading car driving down
a given road. At some point the ego goes under the attack and 
the behaviour will be studied. The scenario ends either via a 
timeout, collision or if the ego vehicle stopped close enough to the leading vehicle
"""

import random
import os
import py_trees
import carla
import operator


from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower,
                                                                      AccelerateToCatchUp,
                                                                      ChangeActorTargetSpeed,
                                                                      RunScript)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (CollisionTest,
                                                                     KeepLaneTest,
                                                                     DrivenDistanceTest,
                                                                     MaxVelocityTest,
                                                                     OutsideRouteLanesTest)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill)
from srunner.tools.route_manipulation import interpolate_trajectory
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


class CyberFollowingNPCStaticFalse(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 30            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=timeout):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """
        self._config = config
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        self.timeout = self._config.scene_params.time_out
        self._base_path = "/home/mohsen/planner.ai/iseauto_ws/src/cyber_security/src/"
        
        super(CyberFollowingNPCStaticFalse, self).__init__("FollowVehicle",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

            # Example code how to randomize start location
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self.other_actors[0].set_transform(waypoint.transform)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._config.scene_params.init_dis_to_npc)
        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
                           first_vehicle_waypoint.transform.rotation)
        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
                           self._other_actor_transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.lincoln.mkz_2017', first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        """

        # to avoid the other actor blocking traffic, it was spawned elsewhere
        # reset its pose to the required one
        # Triggers : RelativeVelocityToOtherActor, InTriggerRegion, InTriggerDistanceToVehicle, InTriggerDistanceToLocation ...
        #               InTriggerDistanceToLocationAlongRoute, WaitUntilInFront, DriveDistance, AtRightmostLane
        # Behaviuors : RunScript, ChangeActorControl, ChangeActorTargetSpeed, ChangeActorWaypoints

        start_transform = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)

        # let the other actor drive until next intersection
        # @todo: We should add some feedback mechanism to respond to ego_vehicle behavior
        driving_to_next_intersection = py_trees.composites.Parallel("DrivingTowardsIntersection",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._config.scene_params.npc_speed))

        attack_sequence = py_trees.composites.Sequence("AttackandIntersectionSequence")
                    
        # Distance to Vehicle Trigger
        attack_sequence.add_child(InTriggerDistanceToVehicle(
                                    self.other_actors[0], self.ego_vehicles[0], self._config.scene_params.attack_place, distance_type="longitudinal"))
                

        # Cyber Attack Trigers
        attack_script_start = "python publish_attack.py" + \
                            " -a {}".format(self._config.scene_params.adversary) + \
                            " -p {}".format(self._config.scene_params.attack_points) + \
                            " -f {}".format(self._config.scene_params.attack_freq) + \
                            " -d {}".format(self._config.scene_params.attack_duration)

        attack_sequence.add_child(RunScript(attack_script_start, base_path=self._base_path)) # Enabling the attack
        # attack_sequence.add_child(DriveDistance(self.ego_vehicles[0], self._config.scene_params.attack_duration)) # duration of the attack

        # end condition Trigger
        # endcondition = py_trees.composites.Parallel("Waiting for end position",
        #                                             policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=25,
                                                        comparison_operator=operator.gt,
                                                        distance_type="longitudinal",
                                                        name="FinalDistance")
        # endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)

        # endcondition_part3 = InTriggerDistanceToNextIntersection(
        #                                 self.other_actors[0], self._other_actor_stop_in_front_intersection)
        # endcondition.add_child(endcondition_part1)
        # endcondition.add_child(endcondition_part2)
        # endcondition.add_child(endcondition_part3)

        attack_sequence.add_child(endcondition_part1)


        driving_to_next_intersection.add_child(attack_sequence)
        # stop vehicle
        stop = StopVehicle(self.other_actors[0], self._other_actor_max_brake)

        
        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform)
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(stop)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """

        # Atomic Criteria : MaxVelocityTest, AverageVelocityTest, OffRoadTest, InRouteTest
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0],terminate_on_failure=True)
        keeplane_criterion = KeepLaneTest(self.ego_vehicles[0])
        drivendistance_criterion = DrivenDistanceTest(self.ego_vehicles[0],100,distance_acceptable=50)
        maxvelocity_criterion = MaxVelocityTest(self.ego_vehicles[0],5.5,optional=True)


        criteria.append(collision_criterion)
        criteria.append(keeplane_criterion)
        criteria.append(drivendistance_criterion)
        criteria.append(maxvelocity_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()




class CyberFollowingNPCFrame(BasicScenario):

    """
    Attack Model: Changing the sensor frame 
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 30            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=timeout):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """
        self._config = config
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        self.timeout = self._config.scene_params.time_out
        self._base_path = "/home/mohsen/planner.ai/iseauto_ws/src/cyber_security/src/"
        
        super(CyberFollowingNPCFrame, self).__init__("FollowVehicle",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

            # Example code how to randomize start location
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self.other_actors[0].set_transform(waypoint.transform)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._config.scene_params.init_dis_to_npc)
        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
                           first_vehicle_waypoint.transform.rotation)
        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
                           self._other_actor_transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.lincoln.mkz_2017', first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """

        """

        # to avoid the other actor blocking traffic, it was spawned elsewhere
        # reset its pose to the required one
        # Triggers : RelativeVelocityToOtherActor, InTriggerRegion, InTriggerDistanceToVehicle, InTriggerDistanceToLocation ...
        #               InTriggerDistanceToLocationAlongRoute, WaitUntilInFront, DriveDistance, AtRightmostLane
        # Behaviuors : RunScript, ChangeActorControl, ChangeActorTargetSpeed, ChangeActorWaypoints

        start_transform = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)

        # let the other actor drive until next intersection
        # @todo: We should add some feedback mechanism to respond to ego_vehicle behavior
        driving_to_next_intersection = py_trees.composites.Parallel("DrivingTowardsIntersection",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._config.scene_params.npc_speed))

        attack_sequence = py_trees.composites.Sequence("AttackandIntersectionSequence")
                    
        # Distance to Vehicle Trigger
        attack_sequence.add_child(InTriggerDistanceToVehicle(
                                    self.other_actors[0], self.ego_vehicles[0], self._config.scene_params.attack_place))
                
        # Cyber Attack Trigers
        attack_script_start = "python change_frame.py" + \
                            " -d {}".format(self._config.scene_params.attack_duration) + \
                            " -l {}".format(self._config.scene_params.attack_side)

        attack_sequence.add_child(RunScript(attack_script_start, base_path=self._base_path)) # Enabling the attack
        attack_sequence.add_child(DriveDistance(self.ego_vehicles[0], self._config.scene_params.attack_duration)) # duration of the attack

        # end condition Trigger
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        comparison_operator=operator.gt,
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)

        endcondition_part3 = InTriggerDistanceToNextIntersection(
                                        self.other_actors[0], self._other_actor_stop_in_front_intersection)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)
        endcondition.add_child(endcondition_part3)

        attack_sequence.add_child(endcondition)


        driving_to_next_intersection.add_child(attack_sequence)
        # stop vehicle
        stop = StopVehicle(self.other_actors[0], self._other_actor_max_brake)

        
        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform)
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(stop)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """

        # Atomic Criteria : MaxVelocityTest, AverageVelocityTest, OffRoadTest, InRouteTest
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0],terminate_on_failure=True)
        keeplane_criterion = KeepLaneTest(self.ego_vehicles[0],optional=True)
        drivendistance_criterion = DrivenDistanceTest(self.ego_vehicles[0],100,distance_acceptable=50,optional=False)
        maxvelocity_criterion = MaxVelocityTest(self.ego_vehicles[0],5.5,optional=True)


        criteria.append(collision_criterion)
        criteria.append(keeplane_criterion)
        criteria.append(drivendistance_criterion)
        criteria.append(maxvelocity_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()




class CyberFollowingNPCFalseRandom(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 30            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=timeout):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """
        self._config = config
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        self.timeout = self._config.scene_params.time_out
        self._base_path = "/home/mohsen/planner.ai/iseauto_ws/src/cyber_security/src/"
        
        super(CyberFollowingNPCFalseRandom, self).__init__("FollowVehicle",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

            # Example code how to randomize start location
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self.other_actors[0].set_transform(waypoint.transform)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._config.scene_params.init_dis_to_npc)
        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
                           first_vehicle_waypoint.transform.rotation)
        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
                           self._other_actor_transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.lincoln.mkz_2017', first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        """

        # to avoid the other actor blocking traffic, it was spawned elsewhere
        # reset its pose to the required one
        # Triggers : RelativeVelocityToOtherActor, InTriggerRegion, InTriggerDistanceToVehicle, InTriggerDistanceToLocation ...
        #               InTriggerDistanceToLocationAlongRoute, WaitUntilInFront, DriveDistance, AtRightmostLane
        # Behaviuors : RunScript, ChangeActorControl, ChangeActorTargetSpeed, ChangeActorWaypoints

        start_transform = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)

        # let the other actor drive until next intersection
        # @todo: We should add some feedback mechanism to respond to ego_vehicle behavior
        driving_to_next_intersection = py_trees.composites.Parallel("DrivingTowardsIntersection",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._config.scene_params.npc_speed))

        attack_sequence = py_trees.composites.Sequence("AttackandIntersectionSequence")
                    
        # Distance to Vehicle Trigger
        attack_sequence.add_child(InTriggerDistanceToVehicle(
                                    self.other_actors[0], self.ego_vehicles[0], self._config.scene_params.attack_place))
                
        if self._config.scene_params.adversary == 'True':

            # Cyber Attack Trigers
            attack_script_start = "python publish_attack.py" + \
                                " -a {}".format(self._config.scene_params.adversary) + \
                                " -p {}".format(self._config.scene_params.attack_points) + \
                                " -f {}".format(self._config.scene_params.attack_freq) + \
                                " -r {}".format(self._config.scene_params.attack_volume)

            attack_script_finish = "python publish_attack.py -a False" 
            attack_sequence.add_child(RunScript(attack_script_start, base_path=self._base_path)) # Enabling the attack
            attack_sequence.add_child(DriveDistance(self.ego_vehicles[0], self._config.scene_params.attack_duration)) # duration of the attack
            attack_sequence.add_child(RunScript(attack_script_finish, base_path=self._base_path)) # Disabling the attack

        # end condition Trigger
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=25,
                                                        comparison_operator=operator.gt,
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)

        endcondition_part3 = InTriggerDistanceToNextIntersection(
                                        self.other_actors[0], self._other_actor_stop_in_front_intersection)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)
        endcondition.add_child(endcondition_part3)

        attack_sequence.add_child(endcondition)


        driving_to_next_intersection.add_child(attack_sequence)
        # stop vehicle
        stop = StopVehicle(self.other_actors[0], self._other_actor_max_brake)

        
        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform)
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(stop)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """

        # Atomic Criteria : MaxVelocityTest, AverageVelocityTest, OffRoadTest, InRouteTest
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0],terminate_on_failure=True)
        keeplane_criterion = KeepLaneTest(self.ego_vehicles[0])
        drivendistance_criterion = DrivenDistanceTest(self.ego_vehicles[0],100,distance_acceptable=50)
        maxvelocity_criterion = MaxVelocityTest(self.ego_vehicles[0],5.5,optional=True)


        criteria.append(collision_criterion)
        criteria.append(keeplane_criterion)
        criteria.append(drivendistance_criterion)
        criteria.append(maxvelocity_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()





class NonCyberFollowingNPC(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 30            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=timeout):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """
        self._config = config
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        self.timeout = self._config.scene_params.time_out
        super(NonCyberFollowingNPC, self).__init__("FollowVehicle",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

            # Example code how to randomize start location
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self.other_actors[0].set_transform(waypoint.transform)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._config.scene_params.init_dis_to_npc)
        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
                           first_vehicle_waypoint.transform.rotation)
        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
                           self._other_actor_transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.lincoln.mkz_2017', first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        """

        # to avoid the other actor blocking traffic, it was spawned elsewhere
        # reset its pose to the required one
        # Triggers : RelativeVelocityToOtherActor, InTriggerRegion, InTriggerDistanceToVehicle, InTriggerDistanceToLocation ...
        #               InTriggerDistanceToLocationAlongRoute, WaitUntilInFront, DriveDistance, AtRightmostLane
        # Behaviuors : RunScript, ChangeActorControl, ChangeActorTargetSpeed, ChangeActorWaypoints

        start_transform = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)

        # let the other actor drive until next intersection
        # @todo: We should add some feedback mechanism to respond to ego_vehicle behavior
        driving_to_next_intersection = py_trees.composites.Parallel("DrivingTowardsIntersection",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._config.scene_params.npc_speed))

        attack_sequence = py_trees.composites.Sequence("AttackandIntersectionSequence")
                    
        # Distance to Vehicle Trigger
        attack_sequence.add_child(InTriggerDistanceToVehicle(
                                    self.other_actors[0], self.ego_vehicles[0], 5, distance_type="longitudinal"))
        attack_sequence.add_child(InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        comparison_operator=operator.gt,
                                                        distance_type="longitudinal",
                                                        name="FinalDistance"))
                

        # end condition Trigger
        # endcondition = py_trees.composites.Parallel("Waiting for end position",
        #                                             policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
        #                                                 self.ego_vehicles[0],
        #                                                 distance=15,
        #                                                 comparison_operator=operator.gt,
        #                                                 name="FinalDistance")
        # endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)

        # endcondition_part3 = InTriggerDistanceToNextIntersection(
        #                                 self.other_actors[0], self._other_actor_stop_in_front_intersection)
        # endcondition.add_child(endcondition_part1)
        # # endcondition.add_child(endcondition_part2)
        # endcondition.add_child(endcondition_part3)

        # attack_sequence.add_child(endcondition)


        driving_to_next_intersection.add_child(attack_sequence)
        # stop vehicle
        stop = StopVehicle(self.other_actors[0], self._other_actor_max_brake)

        
        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform)
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(stop)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """

        # Atomic Criteria : MaxVelocityTest, AverageVelocityTest, OffRoadTest, InRouteTest
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0],terminate_on_failure=True)
        keeplane_criterion = KeepLaneTest(self.ego_vehicles[0])
        drivendistance_criterion = DrivenDistanceTest(self.ego_vehicles[0],100,distance_acceptable=50)
        maxvelocity_criterion = MaxVelocityTest(self.ego_vehicles[0],5.5,optional=True)


        criteria.append(collision_criterion)
        criteria.append(keeplane_criterion)
        criteria.append(drivendistance_criterion)
        criteria.append(maxvelocity_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()