

class Vector3:
    def __init__(self,x=0,y=0,z=0, vec_as_list= None):
        if vec_as_list is not None:
            if len(vec_as_list) != 3:
                raise Exception("List not length 3")
            else:
                self.x = vec_as_list[0]
                self.y = vec_as_list[1]
                self.z = vec_as_list[2]
        else:
            self.x = x
            self.y = y
            self.z = z

    def __add__(self, other):
        if isinstance(other,Vector3):
            return Vector3(self.x + other.x,self.y+other.y,self.z+other.z)


    def __idiv__(self, other):
        pass

    def __xor__(self, other):
        pass


    def __iadd__(self, other):
        self.x += other.x
        self.y +=other.y
        self.z += other.z
        return self

