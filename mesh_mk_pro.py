import numpy as np
import numpy as np
import open3d as o3d

def main():
    #circle
    r1 = 1.0
    r2 = 2.0
    RAD = np.linspace(0, 0.5*np.pi, 20)
    vec = []
    for rad in RAD:
        vec = vec + [[r*np.cos(rad), r*np.sin(rad), r] for r in np.linspace(r1, r2, 10)]
    #for i in range(5):
    #    vec = vec + [[i, j, j] for j in range(5)]
    pcd = mk_pcd(vec)
    
    #o3d.visualization.draw_geometries([pcd])
    o3d.io.write_point_cloud('point.pcd', pcd, write_ascii=False, compressed =False, print_progress=False)
    mesh = create_mesh(pcd)
    o3d.visualization.draw_geometries([mesh])
    write_mesh('FARO_kokubunken.ply',mesh)
    return

def mk_pcd(vec):
    #vec = np.empty((0,3))
    #col = np.empty((0,3))
    #vec,col = add_points(filename,vec,col)
    min = np.amin(vec, axis=0)
    max = np.amax(vec, axis=0)
    vec = vec - min
    #bbox = max-min
    #print(f"size: {bbox[0]:.1f} x {bbox[1]:.1f} x {bbox[2]:.1f} (m)")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(vec)
    #pcd.colors = o3d.utility.Vector3dVector(col)
    return pcd

def create_mesh(point_cloud):
    point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=2))
    point_cloud.orient_normals_to_align_with_direction(orientation_reference=np.array([0., 0., 1.])) # Upの軸を指定する
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        poisson_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            point_cloud)
    print(poisson_mesh)
    bbox = point_cloud.get_axis_aligned_bounding_box()
    mesh = poisson_mesh.crop(bbox)
    # メッシュの軽量化
    # decimation_ratio = 0.5
    # count = len(mesh.triangles)
    #mesh = mesh.simplify_quadric_decimation(int(count * decimation_ratio))
    return mesh

def write_mesh(filename, mesh):
    o3d.io.write_triangle_mesh(filename, mesh, write_ascii=False, write_vertex_normals =True)

if __name__ == '__main__':
    main()