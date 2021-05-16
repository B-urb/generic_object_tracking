
import pyglet
import pyglet.gl as gl
class Visualizer(pyglet.window.Window):
    def __init__(self,*args,**kwargs):
        super().__init__(*args, **kwargs)
        #pyglet.app.run()


    def on_draw(self):
        self.window.clear()

        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_LINE_SMOOTH)

        width, height = self.get_size()
        gl.glViewport(0, 0, width, height)

        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        gl.gluPerspective(60, width / float(height), 0.01, 20)

        gl.glMatrixMode(gl.GL_TEXTURE)
        gl.glLoadIdentity()
        # texcoords are [0..1] and relative to top-left pixel corner, add 0.5 to center
        gl.glTranslatef(0.5 / image_data.width, 0.5 / image_data.height, 0)
        image_texture = image_data.get_texture()
        # texture size may be increased by pyglet to a power of 2
        tw, th = image_texture.owner.width, image_texture.owner.height
        gl.glScalef(image_data.width / float(tw),
                    image_data.height / float(th), 1)

        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()

        gl.gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0)

        gl.glTranslatef(0, 0, state.distance)
        gl.glRotated(state.pitch, 1, 0, 0)
        gl.glRotated(state.yaw, 0, 1, 0)

        if any(state.mouse_btns):
            axes(0.1, 4)

        gl.glTranslatef(0, 0, -state.distance)
        gl.glTranslatef(*state.translation)

        gl.glColor3f(0.5, 0.5, 0.5)
        gl.glPushMatrix()
        gl.glTranslatef(0, 0.5, 0.5)
        grid()
        gl.glPopMatrix()

        psz = max(window.get_size()) / float(max(w, h)) if state.scale else 1
        gl.glPointSize(psz)
        distance = (0, 0, 1) if state.attenuation else (1, 0, 0)
        gl.glPointParameterfv(gl.GL_POINT_DISTANCE_ATTENUATION,
                              (gl.GLfloat * 3)(*distance))

        if state.lighting:
            ldir = [0.5, 0.5, 0.5]  # world-space lighting
            ldir = np.dot(state.rotation, (0, 0, 1))  # MeshLab style lighting
            ldir = list(ldir) + [0]  # w=0, directional light
            gl.glLightfv(gl.GL_LIGHT0, gl.GL_POSITION, (gl.GLfloat * 4)(*ldir))
            gl.glLightfv(gl.GL_LIGHT0, gl.GL_DIFFUSE,
                         (gl.GLfloat * 3)(1.0, 1.0, 1.0))
            gl.glLightfv(gl.GL_LIGHT0, gl.GL_AMBIENT,
                         (gl.GLfloat * 3)(0.75, 0.75, 0.75))
            gl.glEnable(gl.GL_LIGHT0)
            gl.glEnable(gl.GL_NORMALIZE)
            gl.glEnable(gl.GL_LIGHTING)

        gl.glColor3f(1, 1, 1)
        texture = image_data.get_texture()
        gl.glEnable(texture.target)
        gl.glBindTexture(texture.target, texture.id)
        gl.glTexParameteri(
            gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)

        # comment this to get round points with MSAA on
        gl.glEnable(gl.GL_POINT_SPRITE)

        if not state.scale and not state.attenuation:
            gl.glDisable(gl.GL_MULTISAMPLE)  # for true 1px points with MSAA on
        vertex_list.draw(gl.GL_POINTS)
        gl.glDisable(texture.target)
        if not state.scale and not state.attenuation:
            gl.glEnable(gl.GL_MULTISAMPLE)

        gl.glDisable(gl.GL_LIGHTING)

        gl.glColor3f(0.25, 0.25, 0.25)
        frustum(depth_intrinsics)
        axes()

        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        gl.glOrtho(0, width, 0, height, -1, 1)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        gl.glMatrixMode(gl.GL_TEXTURE)
        gl.glLoadIdentity()
        gl.glDisable(gl.GL_DEPTH_TEST)

        fps_display.draw()

