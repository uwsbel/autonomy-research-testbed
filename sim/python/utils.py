from typing import Tuple, List

import pychrono as chrono
import pychrono.vehicle as veh


def AddConesFromFile(
    system: chrono.ChSystem,
    terrain: veh.ChTerrain,
    path_filename: str,
    cone_offset: Tuple[int, int] = (0, 0),
):
    cone_offset_x, cone_offset_y = cone_offset

    red_cone_mesh_filename = chrono.GetChronoDataFile("sensor/cones/red_cone.obj")
    red_cone_mesh = chrono.ChTriangleMeshConnected()
    red_cone_mesh.LoadWavefrontMesh(red_cone_mesh_filename, False, True)

    green_cone_mesh_filename = chrono.GetChronoDataFile("sensor/cones/green_cone.obj")
    green_cone_mesh = chrono.ChTriangleMeshConnected()
    green_cone_mesh.LoadWavefrontMesh(green_cone_mesh_filename, False, True)

    red_cone_assets, green_cone_assets = [], []
    with open(path_filename) as cone_file:
        while line := cone_file.readline():
            _, color, x, y = map(float, line.split("\t"))

            pos_x = x + cone_offset_x
            pos_y = y + cone_offset_y
            pos_z = terrain.GetHeight(chrono.ChVectorD(pos_x, pos_y, 1000))

            pos = chrono.ChVectorD(pos_x, pos_y, pos_z)
            rot = chrono.ChQuaternionD(1, 0, 0, 0)

            cone_shape = chrono.ChTriangleMeshShape()
            if color == 0:
                cone_shape.SetMesh(red_cone_mesh)
                red_cone_assets.append(cone_shape)
            else:
                cone_shape.SetMesh(green_cone_mesh)
                green_cone_assets.append(cone_shape)

            cone_body = chrono.ChBody()
            cone_body.SetPos(pos)
            cone_body.SetRot(rot)
            cone_body.AddVisualShape(cone_shape)
            cone_body.SetBodyFixed(True)
            system.Add(cone_body)

    return red_cone_assets, green_cone_assets


def LabelAssets(assets: List[chrono.ChTriangleMeshShape], class_id: int):
    instance_id = 0
    for cone in assets:
        instance_id += 1
        for mat in cone.GetMaterials():
            mat.SetClassID(class_id)
            mat.SetInstanceID(instance_id)
