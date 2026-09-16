"""Bounded SI OBJ import for closed, outward-oriented convex triangular solids.

Only convex single connected shells are supported. Convex supporting planes,
closed manifold topology, and non-overlapping coplanar triangles exclude
self-intersection; concave CAD must first be decomposed into separate solids.
Separate buoyancy solids additionally require strictly separated AABBs. This
conservative restriction can reject valid nearby hulls and never admits overlap.
"""
from collections import Counter, defaultdict
import math
from pathlib import Path


def sub(a, b):
    return [x-y for x,y in zip(a,b)]


def dot(a, b):
    return sum(x*y for x,y in zip(a,b))


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def _overlap_area(first, second, normal):
    """Intersection area in a stable planar projection via convex clipping."""
    drop=max(range(3),key=lambda i:abs(normal[i]))
    first=[[v[i] for i in range(3) if i!=drop] for v in first]
    second=[[v[i] for i in range(3) if i!=drop] for v in second]
    def side(a,b,p): return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
    sign=1 if side(*second)>0 else -1
    poly=first
    for a,b in zip(second,second[1:]+second[:1]):
        output=[]
        for p,q in zip(poly,poly[1:]+poly[:1]):
            sp,sq=sign*side(a,b,p),sign*side(a,b,q)
            if sp>=0: output.append(p)
            if (sp<0<=sq) or (sq<0<=sp):
                t=sp/(sp-sq)
                output.append([p[i]+t*(q[i]-p[i]) for i in range(2)])
        poly=output
        if not poly: return 0.
    return abs(sum(p[0]*q[1]-p[1]*q[0] for p,q in zip(poly,poly[1:]+poly[:1])))/2


def validate_convex_mesh(vertices, faces):
    """Return volume in m³, rejecting unsupported or invalid triangle shells."""
    if len(vertices)<4 or len(faces)<4:
        raise ValueError('mesh requires at least four vertices and triangles')
    if len(vertices)>10000 or len(faces)>20000:
        raise ValueError('mesh exceeds bounded importer size; simplify prepared convex hull')
    for v in vertices:
        if len(v)!=3 or any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in v):
            raise ValueError('mesh vertices must be finite SI xyz coordinates')
    if len(set(map(tuple,vertices)))!=len(vertices):
        raise ValueError('mesh duplicate vertices must be welded before import')
    scale=max(max(v[i] for v in vertices)-min(v[i] for v in vertices) for i in range(3))
    if scale<=0: raise ValueError('mesh must have nonzero extent')
    eps=scale*1e-9
    # Center arithmetic to avoid cancellation on resources with large origins.
    center=[sum(v[i] for v in vertices)/len(vertices) for i in range(3)]
    points=[sub(v,center) for v in vertices]
    edges=Counter(); adjacency=defaultdict(set); used=set(); normals=[]; volume=0.
    seen_faces=set()
    for face in faces:
        if len(face)!=3 or any(type(i) is not int or not 0<=i<len(vertices) for i in face) or len(set(face))!=3:
            raise ValueError('mesh faces must be valid nondegenerate triangle indices')
        key=tuple(sorted(face))
        if key in seen_faces: raise ValueError('mesh contains duplicate triangle')
        seen_faces.add(key)
        a,b,c=[points[i] for i in face]
        n=cross(sub(b,a),sub(c,a)); length=math.sqrt(dot(n,n))
        if length <= eps*scale: raise ValueError('mesh contains degenerate triangle')
        n=[x/length for x in n]
        if any(dot(n,sub(p,a))>eps for p in points):
            raise ValueError('mesh must be convex and outward oriented; concave or self-intersecting resource rejected')
        normals.append(n)
        volume+=dot(a,cross(b,c))/6
        used.update(face)
        for i,j in zip(face,face[1:]+face[:1]):
            edges[i,j]+=1; adjacency[i].add(j); adjacency[j].add(i)
    if used!=set(range(len(vertices))): raise ValueError('mesh contains unused vertices')
    if any(count!=1 or edges[j,i]!=1 for (i,j),count in edges.items()):
        raise ValueError('mesh must be watertight with consistently oriented manifold edges')
    visited=set(); pending=[0]
    while pending:
        i=pending.pop()
        if i in visited: continue
        visited.add(i); pending.extend(adjacency[i]-visited)
    if len(visited)!=len(vertices) or len(vertices)-len(edges)//2+len(faces)!=2:
        raise ValueError('mesh must be one connected genus-zero solid')
    for i,face in enumerate(faces):
        a=points[face[0]]
        for j in range(i):
            other=faces[j]
            if dot(normals[i],normals[j])>1-1e-9 and all(abs(dot(normals[i],sub(points[k],a)))<=eps for k in other):
                if _overlap_area([points[k] for k in face],[points[k] for k in other],normals[i])>eps*scale:
                    raise ValueError('mesh has overlapping coplanar triangles / self-intersection')
    if volume<=eps**3: raise ValueError('mesh requires positive enclosed volume')
    return volume


def load_obj(path):
    """Read pre-scaled metre vertices and triangular OBJ faces (1-based indices)."""
    vertices=[]; faces=[]
    for lineno,line in enumerate(Path(path).read_text().splitlines(),1):
        words=line.split('#',1)[0].split()
        if not words: continue
        try:
            if words[0]=='v' and len(words)==4:
                vertices.append([float(v) for v in words[1:]])
            elif words[0]=='f' and len(words)==4:
                face=[int(v.split('/')[0]) for v in words[1:]]
                if any(v<=0 for v in face): raise ValueError('positive absolute OBJ indices required')
                faces.append([v-1 for v in face])
            elif words[0] in {'o','g','s','vn','vt','usemtl','mtllib'}:
                # Materials are deliberately not loaded: no untracked resources.
                if words[0] in {'usemtl','mtllib'}: raise ValueError('external OBJ material resources unsupported')
            else:
                raise ValueError('only xyz vertices and triangular faces supported')
        except ValueError as error:
            raise ValueError(f'{path}:{lineno}: {error}') from error
    return vertices,faces,validate_convex_mesh(vertices,faces)


def geometry_mesh(geometry):
    """Return triangle mesh in body coordinates for an accepted geometry."""
    if geometry['type']=='mesh':
        vertices=geometry['vertices']; faces=geometry['faces']
    else:
        x,y,z=[v/2 for v in geometry['size_m']]
        vertices=[[-x,-y,-z],[x,-y,-z],[x,y,-z],[-x,y,-z],[-x,-y,z],[x,-y,z],[x,y,z],[-x,y,z]]
        faces=[[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
    return [[v[i]+geometry['pose'][i] for i in range(3)] for v in vertices], faces


def geometry_vertices(geometry):
    return geometry_mesh(geometry)[0]


def geometry_volume(geometry):
    return geometry['volume_m3'] if geometry['type']=='mesh' else math.prod(geometry['size_m'])


def validate_disjoint_volumes(volumes):
    bounds=[]
    for volume in volumes:
        vertices=geometry_vertices(volume)
        bounds.append([(min(v[i] for v in vertices),max(v[i] for v in vertices)) for i in range(3)])
    for i,a in enumerate(bounds):
        for b in bounds[:i]:
            if not any(a[k][1]<b[k][0]-1e-9 or b[k][1]<a[k][0]-1e-9 for k in range(3)):
                raise ValueError('buoyancy volumes overlap or touch conservative AABBs; separate hull volumes required')
