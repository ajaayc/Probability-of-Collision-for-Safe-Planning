#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/mcsimplugin')
try:
    env=Environment()
    import pdb
    pdb.set_trace()
    env.Load('scenes/myscene.env.xml')
    MCModule = RaveCreateModule(env,'MCModule')
    print MCModule.SendCommand('help')
    print "Python about to send armacommand"
    print MCModule.SendCommand('ArmaCommand')
    print "Python finished sending armacommand"
    print MCModule.SendCommand('MyCommand')

finally:
    RaveDestroy()
