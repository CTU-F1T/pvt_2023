#!/usr/bin/env bash

set -ex

ros2 topic pub /eStop -1 --keep-alive 5 std_msgs/msg/Bool 'data: False'
