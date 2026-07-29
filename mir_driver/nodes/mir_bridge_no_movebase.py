#!/usr/bin/env python3
# Copyright (c) 2018-2022, Martin Guenther (DFKI GmbH) and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Martin Guenther
#
# --- Modified: move_base forwarding removed (no local move_base client,
#     no move_base_msgs topics, no MirMoveBase action goal translation). ---

import rospy

import copy
import sys
from collections.abc import Iterable

from mir_driver import rosbridge
from rospy_message_converter import message_converter

import actionlib_msgs.msg
import diagnostic_msgs.msg
import dynamic_reconfigure.msg
import geometry_msgs.msg
import mir_msgs.msg
import nav_msgs.msg
import rosgraph_msgs.msg
import sdc21x0.msg
import sensor_msgs.msg
import std_msgs.msg
import tf2_msgs.msg
import visualization_msgs.msg

from collections import OrderedDict

tf_prefix = ''
static_transforms = OrderedDict()


class TopicConfig(object):
    def __init__(self, topic, topic_type, latch=False, dict_filter=None):
        self.topic = topic
        self.topic_type = topic_type
        self.latch = latch
        self.dict_filter = dict_filter


def _cmd_vel_dict_filter(msg_dict):
    """
    Convert Twist to TwistStamped.

    Convert a geometry_msgs/Twist message dict (as sent from the ROS side) to
    a geometry_msgs/TwistStamped message dict (as expected by the MiR on
    software version >=2.7).
    """
    header = std_msgs.msg.Header(frame_id='', stamp=rospy.Time.now())
    filtered_msg_dict = {
        'header': message_converter.convert_ros_message_to_dictionary(header),
        'twist': copy.deepcopy(msg_dict),
    }
    return filtered_msg_dict


def _tf_dict_filter(msg_dict):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    for transform in filtered_msg_dict['transforms']:
        transform['child_frame_id'] = tf_prefix + '/' + transform['child_frame_id'].strip('/')
    return filtered_msg_dict


def _tf_static_dict_filter(msg_dict):
    """
    Cache tf_static messages (simulate latching).

    The tf_static topic needs special handling. Publishers on tf_static are *latched*, which means that the ROS master
    caches the last message that was sent by each publisher on that topic, and will forward it to new subscribers.
    However, since the mir_driver node appears to the ROS master as a single node with a single publisher on tf_static,
    and there are multiple actual publishers hiding behind it on the MiR side, only one of those messages will be
    cached. Therefore, we need to implement the caching ourselves and make sure that we always publish the full set of
    transforms as a single message.
    """
    global static_transforms

    # prepend tf_prefix
    filtered_msg_dict = _tf_dict_filter(msg_dict)

    # The following code was copied + modified from https://github.com/tradr-project/static_transform_mux .

    # Process the incoming transforms, merge them with our cache.
    for transform in filtered_msg_dict['transforms']:
        key = transform['child_frame_id']
        rospy.loginfo(
            "[%s] tf_static: updated transform %s->%s.",
            rospy.get_name(),
            transform['header']['frame_id'],
            transform['child_frame_id'],
        )
        static_transforms[key] = transform

    # Return the cached messages.
    filtered_msg_dict['transforms'] = static_transforms.values()
    rospy.loginfo(
        "[%s] tf_static: sent %i transforms: %s",
        rospy.get_name(),
        len(static_transforms),
        str(static_transforms.keys()),
    )
    return filtered_msg_dict


def _prepend_tf_prefix_dict_filter(msg_dict):
    if not isinstance(msg_dict, dict):  # can happen during recursion
        return
    for key, value in msg_dict.items():
        if key == 'header':
            try:
                frame_id = value['frame_id'].strip('/')
                if frame_id != 'map':
                    value['frame_id'] = (tf_prefix + '/' + frame_id).strip('/')
                else:
                    value['frame_id'] = frame_id
            except TypeError:
                pass
            except KeyError:
                pass
        elif isinstance(value, dict):
            _prepend_tf_prefix_dict_filter(value)
        elif isinstance(value, Iterable):
            for item in value:
                _prepend_tf_prefix_dict_filter(item)
    return msg_dict


def _remove_tf_prefix_dict_filter(msg_dict):
    if not isinstance(msg_dict, dict):  # can happen during recursion
        return
    for key, value in msg_dict.items():
        if key == 'header':
            try:
                s = value['frame_id'].strip('/')
                if s.find(tf_prefix) == 0:
                    value['frame_id'] = (s[len(tf_prefix):]).strip('/')
            except TypeError:
                pass
            except KeyError:
                pass
        elif isinstance(value, dict):
            _remove_tf_prefix_dict_filter(value)
        elif isinstance(value, Iterable):
            for item in value:
                _remove_tf_prefix_dict_filter(item)
    return msg_dict


# Topics forwarded MiR -> ROS. Trimmed down to what a Nav2 localization/
# driving stack actually needs: no move_base_node/*, no move_base/*,
# no MissionController/CheckArea marker, no camera_floor point clouds.
# Uncomment any line back in if you find you need it.
PUB_TOPICS = [
    TopicConfig('LightCtrl/us_list', sensor_msgs.msg.Range),
    TopicConfig('MC/currents', sdc21x0.msg.MotorCurrents),
    TopicConfig('SickPLC/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    TopicConfig('SickPLC/parameter_updates', dynamic_reconfigure.msg.Config),
    TopicConfig('amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped),
    TopicConfig('b_raw_scan', sensor_msgs.msg.LaserScan),
    TopicConfig('b_scan', sensor_msgs.msg.LaserScan),
    # TopicConfig('camera_floor/background', sensor_msgs.msg.PointCloud2),
    # TopicConfig('camera_floor/depth/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('camera_floor/depth/parameter_updates', dynamic_reconfigure.msg.Config),
    # TopicConfig('camera_floor/depth/points', sensor_msgs.msg.PointCloud2),
    # TopicConfig('camera_floor/filter/visualization_marker', visualization_msgs.msg.Marker),
    # TopicConfig('camera_floor/floor', sensor_msgs.msg.PointCloud2),
    # TopicConfig('camera_floor/obstacles', sensor_msgs.msg.PointCloud2),
    TopicConfig('diagnostics', diagnostic_msgs.msg.DiagnosticArray),
    TopicConfig('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray),
    TopicConfig('diagnostics_toplevel_state', diagnostic_msgs.msg.DiagnosticStatus),
    TopicConfig('f_raw_scan', sensor_msgs.msg.LaserScan),
    TopicConfig('f_scan', sensor_msgs.msg.LaserScan),
    TopicConfig('imu_data', sensor_msgs.msg.Imu),
    TopicConfig('laser_back/driver/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    TopicConfig('laser_back/driver/parameter_updates', dynamic_reconfigure.msg.Config),
    TopicConfig('laser_front/driver/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    TopicConfig('laser_front/driver/parameter_updates', dynamic_reconfigure.msg.Config),
    TopicConfig('/map', nav_msgs.msg.OccupancyGrid, latch=True),
    TopicConfig('/map_metadata', nav_msgs.msg.MapMetaData),
    TopicConfig('mir_amcl/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    TopicConfig('mir_amcl/parameter_updates', dynamic_reconfigure.msg.Config),
    TopicConfig('mir_amcl/selected_points', sensor_msgs.msg.PointCloud2),
    TopicConfig('mir_log', rosgraph_msgs.msg.Log),
    TopicConfig('mir_status_msg', std_msgs.msg.String),
    TopicConfig('mirwebapp/grid_map_metadata', mir_msgs.msg.LocalMapStat),
    TopicConfig('mirwebapp/laser_map_metadata', mir_msgs.msg.LocalMapStat),
    TopicConfig('odom', nav_msgs.msg.Odometry),
    TopicConfig('odom_enc', nav_msgs.msg.Odometry),
    TopicConfig('robot_mode', mir_msgs.msg.RobotMode),
    TopicConfig('robot_pose', geometry_msgs.msg.Pose),
    TopicConfig('robot_state', mir_msgs.msg.RobotState),
    TopicConfig('/rosout', rosgraph_msgs.msg.Log),
    TopicConfig('/rosout_agg', rosgraph_msgs.msg.Log),
    TopicConfig('scan', sensor_msgs.msg.LaserScan),
    TopicConfig('scan_filter/visualization_marker', visualization_msgs.msg.Marker),
    TopicConfig('/tf', tf2_msgs.msg.TFMessage, dict_filter=_tf_dict_filter),
    TopicConfig('/tf_static', tf2_msgs.msg.TFMessage, dict_filter=_tf_static_dict_filter, latch=True),
]

# Topics forwarded ROS -> MiR. move_base/goal and move_base/cancel removed
# since there's no local move_base client anymore.
SUB_TOPICS = [
    TopicConfig('cmd_vel', geometry_msgs.msg.Twist, dict_filter=_cmd_vel_dict_filter),
    TopicConfig('initialpose', geometry_msgs.msg.PoseWithCovarianceStamped),
    TopicConfig('light_cmd', std_msgs.msg.String),
    TopicConfig('mir_cmd', std_msgs.msg.String),
]


class PublisherWrapper(rospy.SubscribeListener):
    def __init__(self, topic_config, robot):
        self.topic_config = topic_config
        self.robot = robot
        self.connected = False
        self.pub = rospy.Publisher(
            name=topic_config.topic,
            data_class=topic_config.topic_type,
            subscriber_listener=self,
            latch=topic_config.latch,
            queue_size=10,
        )
        rospy.loginfo(
            "[%s] publishing topic '%s' [%s]", rospy.get_name(), topic_config.topic, topic_config.topic_type._type
        )
        if topic_config.latch:
            self.peer_subscribe("", None, None)

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        if not self.connected:
            self.connected = True
            rospy.loginfo("[%s] starting to stream messages on topic '%s'", rospy.get_name(), self.topic_config.topic)
            absolute_topic = '/' + self.topic_config.topic.lstrip('/')
            self.robot.subscribe(topic=absolute_topic, callback=self.callback)

    def peer_unsubscribe(self, topic_name, num_peers):
        pass

    def callback(self, msg_dict):
        msg_dict = _prepend_tf_prefix_dict_filter(msg_dict)
        if self.topic_config.dict_filter is not None:
            msg_dict = self.topic_config.dict_filter(msg_dict)
        msg = message_converter.convert_dictionary_to_ros_message(self.topic_config.topic_type._type, msg_dict)
        self.pub.publish(msg)


class SubscriberWrapper(object):
    def __init__(self, topic_config, robot):
        self.topic_config = topic_config
        self.robot = robot
        self.sub = rospy.Subscriber(
            name=topic_config.topic, data_class=topic_config.topic_type, callback=self.callback, queue_size=10
        )
        rospy.loginfo(
            "[%s] subscribing to topic '%s' [%s]", rospy.get_name(), topic_config.topic, topic_config.topic_type._type
        )

    def callback(self, msg):
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        msg_dict = _remove_tf_prefix_dict_filter(msg_dict)
        if self.topic_config.dict_filter is not None:
            msg_dict = self.topic_config.dict_filter(msg_dict)
        absolute_topic = '/' + self.topic_config.topic.lstrip('/')
        self.robot.publish(absolute_topic, msg_dict)


class MiRBridge(object):
    def __init__(self):
        try:
            hostname = rospy.get_param('~hostname')
        except KeyError:
            rospy.logfatal('[%s] parameter "hostname" is not set!', rospy.get_name())
            sys.exit(-1)
        port = rospy.get_param('~port', 9090)

        global tf_prefix
        tf_prefix = rospy.get_param('~tf_prefix', '').strip('/')

        rospy.loginfo('[%s] trying to connect to %s:%i...', rospy.get_name(), hostname, port)
        self.robot = rosbridge.RosbridgeSetup(hostname, port)

        r = rospy.Rate(10)
        i = 1
        while not self.robot.is_connected():
            if rospy.is_shutdown():
                sys.exit(0)
            if self.robot.is_errored():
                rospy.logfatal('[%s] connection error to %s:%i, giving up!', rospy.get_name(), hostname, port)
                sys.exit(-1)
            if i % 10 == 0:
                rospy.logwarn('[%s] still waiting for connection to %s:%i...', rospy.get_name(), hostname, port)
            i += 1
            r.sleep()

        rospy.loginfo('[%s] ... connected.', rospy.get_name())

        topics = self.get_topics()
        published_topics = [topic_name for (topic_name, _, has_publishers, _) in topics if has_publishers]
        subscribed_topics = [topic_name for (topic_name, _, _, has_subscribers) in topics if has_subscribers]

        for pub_topic in PUB_TOPICS:
            PublisherWrapper(pub_topic, self.robot)
            absolute_topic = '/' + pub_topic.topic.lstrip('/')
            if absolute_topic not in published_topics:
                rospy.logwarn("[%s] topic '%s' is not published by the MiR!", rospy.get_name(), pub_topic.topic)

        for sub_topic in SUB_TOPICS:
            SubscriberWrapper(sub_topic, self.robot)
            absolute_topic = '/' + sub_topic.topic.lstrip('/')
            if absolute_topic not in subscribed_topics:
                rospy.logwarn("[%s] topic '%s' is not yet subscribed to by the MiR!", rospy.get_name(), sub_topic.topic)

        # No move_base client / move_base_simple/goal forwarding here on purpose:
        # this bridge intentionally does not drive the MiR's onboard planner.

    def get_topics(self):
        srv_response = self.robot.callService('/rosapi/topics', msg={})
        topic_names = sorted(srv_response['topics'])
        topics = []

        for topic_name in topic_names:
            srv_response = self.robot.callService("/rosapi/topic_type", msg={'topic': topic_name})
            topic_type = srv_response['type']

            srv_response = self.robot.callService("/rosapi/publishers", msg={'topic': topic_name})
            has_publishers = True if len(srv_response['publishers']) > 0 else False

            srv_response = self.robot.callService("/rosapi/subscribers", msg={'topic': topic_name})
            has_subscribers = True if len(srv_response['subscribers']) > 0 else False

            topics.append([topic_name, topic_type, has_publishers, has_subscribers])

        print('Publishers:')
        for topic_name, topic_type, has_publishers, has_subscribers in topics:
            if has_publishers:
                print((' * %s [%s]' % (topic_name, topic_type)))

        print('\nSubscribers:')
        for topic_name, topic_type, has_publishers, has_subscribers in topics:
            if has_subscribers:
                print((' * %s [%s]' % (topic_name, topic_type)))

        return topics


def main():
    rospy.init_node('mir_bridge')
    MiRBridge()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
