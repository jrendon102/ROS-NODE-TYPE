#!/usr/bin/env python3
import rospy


def get_param(param_name: str):
    rospy.logdebug(f"{get_param.__name__}()::Fetching fan parameter:[{param_name}]")
    try:
        return rospy.get_param(param_name)
    except KeyError as exc:
        rospy.logdebug(f"{get_param.__name__}()::Failed to retrieve parameter:[{param_name}]")
        raise rospy.ROSException from exc
