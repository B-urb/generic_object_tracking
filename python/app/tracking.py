def processObjects(objects):
    # clean up dead objects
    for obj in objects:
        obj.alive_frames += 1
        if obj.state_counter < 7 and obj.alive_frames > 10 and obj.status == "candidate":
            objects.remove(obj)
        elif obj.status == "candidate" and obj.state_counter > 10:
            obj.updateState("active")
        if obj.state_counter > 15:
            if obj.status == "lost":
                objects.remove(obj)
            elif obj.status == "active":
                obj.updateState("lost")
    return objects
