import scriptcontext
import ghpythonremote
import Rhino.Geometry as rg

np = scriptcontext.sticky['numpy']
cvxopt = scriptcontext.sticky['cvxopt']
rpy = scriptcontext.sticky['rpy']


# input cell index
# output open polyline of the cell
def get_cell(cell_index):
    start_edge_index = Face_StartEdge[cell_index]
    edge_index = start_edge_index
    # edge_start_index = Edge_Start[start_edge_index]
    # next_edge_index = Edge_Next[start_edge_index]
    polyline = rg.Polyline()
    while True:
        polyline.Add(Vertices[Edge_Start[edge_index]])
        edge_index = Edge_Next[edge_index]
        if edge_index == start_edge_index:
            break
    return polyline


# input cell index
# output open polyline of the cell
def get_fabricated_cell(cell_index):
    polyline = get_cell(cell_index)
    for i in range(polyline.Count):
        polyline[i] += Noise_Vectors.Branch(cell_index)[i]
    return polyline


# input edge index
# output edge line
def get_edge(edge_index):
    start = Vertices[Edge_Start[edge_index]]
    end = Vertices[Edge_Start[Edge_Next[edge_index]]]
    return rg.Line(start, end)


# fix all the bottom edges
def fixed_edges_init():
    edge_indices = []
    for edge_index in range(len(Edge_Start)):
        if Edge_Face[edge_index] != -1:
            continue
        if Vertices[Edge_Start[edge_index]].Y > 0.01:
            continue
        if Vertices[Edge_Start[Edge_Next[edge_index]]].Y > 0.01:
            continue
        edge_indices.append(edge_index)
    return edge_indices




fixed_edges_indices = fixed_edges_init()
a = []
b = []
for face_index in range(1):
    a.append(get_fabricated_cell(face_index).ToPolylineCurve())
for edge_index in fixed_edges_indices:
    b.append(get_edge(edge_index))
