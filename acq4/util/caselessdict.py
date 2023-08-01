
class CaselessDict(dict):
    """Case-insensitive dict. Values can be set and retrieved using keys of any case.
    Note that when iterating, the original case is returned for each key."""
    def __init__(self, *args):
        dict.__init__(self, {}) ## requirement for the empty {} here seems to be a python bug?
        self.keyMap = dict([(k.lower(), k) for k in dict.keys(self)])
        if len(args) == 0:
            return
        elif len(args) == 1 and isinstance(args[0], dict):
            for k in args[0]:
                self[k] = args[0][k]
        else:
            raise Exception("CaselessDict may only be instantiated with a single dict.")
        
    #def keys(self):
        #return self.keyMap.values()
    
    def __setitem__(self, key, val):
        kl = key.lower()
        if kl in self.keyMap:
            dict.__setitem__(self, self.keyMap[kl], val)
        else:
            dict.__setitem__(self, key, val)
            self.keyMap[kl] = key
            
    def __getitem__(self, key):
        kl = key.lower()
        if kl not in self.keyMap:
            raise KeyError(key)
        return Dict.__getitem__(self, self.keyMap[kl])
        
    def __contains__(self, key):
        return key.lower() in self.keyMap
    
    def update(self, d):
        for k, v in d.items():
            self[k] = v
            
    def copy(self):
        return CaselessDict(Dict.copy(self))
        
    def __delitem__(self, key):
        kl = key.lower()
        if kl not in self.keyMap:
            raise KeyError(key)
        dict.__delitem__(self, self.keyMap[kl])
        del self.keyMap[kl]
            
    def __deepcopy__(self, memo):
        raise Exception("deepcopy not implemented")

    def clear(self):
        dict.clear(self)
        self.keyMap.clear()

if __name__ == "__main__":
    cd = CaselessDict()
    print(cd)
