class ServoDict:
    def __init__(self, base_dict):
        self.container = base_dict

    def __getitem__(self, key):
        if isinstance(key, tuple) or isinstance(key, list):
            if len(key) == 2:
                for item in self.container.values():
                    if item['id'] == key[0] and item['sio'] == key[1]:
                        return item
            else:
                raise KeyError
        else: 
            return self.container[key]
    
    def __setitem__(self, key, item):
        self.container[key] = item

    def __iter__(self):
        return iter(self.container)

    def __contains__(self, item):
        return self.container.__contains__(item)
            
    def __getattr__(self, attr):
        if attr == 'ids':
            values = self.container.values 
            return [(value['id'], value['sio']) for value in values]

    def __repr__(self):
        return self.container

if __name__ == "__main__":
    d = {'a': 0, 'b': 1}
    servos = ServoDict(d)
    for servo in servos:
        print(servo, servos[servo])