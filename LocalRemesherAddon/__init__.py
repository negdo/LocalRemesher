'''
Copyright (C) 2023 Miha Marinko
miha.marinko20@gmail.com

Created by Miha Marinko

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''


bl_info = {
    "name": "Local Remesher",
    "description": "Remesh parts of the mesh",
    "author": "Miha Marinko",
    "version": (0, 1, 0),
    "blender": (3, 5, 0),
    "location": "3D Viewport > Sidebar > Local Remesher",
    "warning": "",
    "wiki_url": "",
    "category": "Mesh",
    "bl_options": {"REGISTER", "UNDO"}}


import bpy
from .UI import *
from .GridRemesher import *
from .ParticleRemesher import *


def register():
    bpy.utils.register_class(LocalRemesherPanel)
    bpy.utils.register_class(GridRemesher)
    bpy.utils.register_class(ParticleRemesher)


def unregister():
    bpy.utils.unregister_class(LocalRemesherPanel)
    bpy.utils.unregister_class(GridRemesher)
    bpy.utils.unregister_class(ParticleRemesher)


if __name__ == "__main__":
    register()