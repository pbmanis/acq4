# -*- coding: utf-8 -*-

from lib.util.MetaArray import MetaArray as MA

class MetaArray:
    def __init__(self, data):
        self.data = data
        
    def typeName(self):
        return 'MetaArray'
        
    def write(self, fileName):
        self.data.write(fileName)
        
def fromFile(fileName, info=None):
    return MA(file=fileName)
    