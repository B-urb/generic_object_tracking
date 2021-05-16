

class Vector2:
    def __init__(self,x=0,y=0, vec_as_list= None):
        if vec_as_list is not None:
            if len(vec_as_list) != 2:
                raise Exception("List not length 2")
            else:
                self.x = vec_as_list[0]
                self.y = vec_as_list[1]

        else:
            self.x = x
            self.y= y


    def __add__(self, other):
        if isinstance(other,Vector2):
            return Vector2(self.x + other.x,self.y+other.y)

    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y
        return self

