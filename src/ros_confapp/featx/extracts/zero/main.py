#!/usr/bin/env python
from  import Navigation

class RootBot(Navigation):
    def __init__(self):
        print("Root Bot")
        Navigation.__init__(self)