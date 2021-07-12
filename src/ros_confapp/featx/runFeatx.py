#!/usr/bin/env python
from ros_confapp.featx.featxCore import FeatxCore

featx = FeatxCore()
featx.loadControl()
print("-----------------------")
featx.loadNavigation()