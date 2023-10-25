#!/bin/bash
rosservice call /mavros/cmd/command "{broadcast: false, command: 246, confirmation: true, param1: 1, param2: 0}"
