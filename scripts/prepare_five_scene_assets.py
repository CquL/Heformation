#!/usr/bin/env python3
"""Convert already-vendored Stonefish display assets to RViz COLLADA.

Offline format conversion only. No physics, controller or new online node.
Source OBJ, texture, GPL license and source hashes accompany the generated mesh.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def world_models(output, scene):
    """Build the approved scene's meshes and major-structure box proxies once.

    Detailed meshes are display resources; conservative AABBs use the existing
    sensing/clearance model. Both come from these same component dimensions.
    No controller, task assignment, dynamic scene authority or simulator is
    introduced by this offline asset preparation.
    """
    white=(.88,.91,.92,1.);steel=(.25,.32,.35,1.)
    blue=(.035,.18,.33,1.);glass=(.08,.39,.51,1.)
    yellow=(.96,.65,.12,1.);orange=(.9,.29,.06,1.)
    stone=(.38,.42,.39,1.);wood=(.39,.33,.24,1.)
    models=[];proxies=[];records={}

    for spec in scene.get('world_models',()):
        kind=spec['model'];parts={};local_boxes=[]

        def polygon(points, colour):
            for i in range(1,len(points)-1):
                a,b,c=points[0],points[i],points[i+1]
                u=[b[j]-a[j] for j in range(3)];v=[c[j]-a[j] for j in range(3)]
                n=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
                length=math.sqrt(sum(x*x for x in n))
                if length>1e-10:parts.setdefault(colour,[]).append((a,b,c,tuple(x/length for x in n)))

        def proxy(name, points):
            low=[min(p[j] for p in points) for j in range(3)]
            high=[max(p[j] for p in points) for j in range(3)]
            local_boxes.append((name,[(a+b)/2 for a,b in zip(low,high)],
                                [max(.04,b-a) for a,b in zip(low,high)]))

        def box(name, center, size, colour, solid=False):
            x,y,z=center;a,b,c=[v/2 for v in size]
            p=[(x+dx*a,y+dy*b,z+dz*c) for dx,dy,dz in
               ((-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
                (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1))]
            for face in ((0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)):
                polygon([p[i] for i in face],colour)
            if solid:proxy(name,p)

        def tube(name, a, b, r, colour, solid=False, segments=18, end_r=None):
            axis=tuple(y-x for x,y in zip(a,b));length=math.sqrt(sum(x*x for x in axis))
            w=tuple(x/length for x in axis);seed=(0.,0.,1.) if abs(w[2])<.9 else (1.,0.,0.)
            u=(w[1]*seed[2]-w[2]*seed[1],w[2]*seed[0]-w[0]*seed[2],w[0]*seed[1]-w[1]*seed[0])
            norm=math.sqrt(sum(x*x for x in u));u=tuple(x/norm for x in u)
            v=(w[1]*u[2]-w[2]*u[1],w[2]*u[0]-w[0]*u[2],w[0]*u[1]-w[1]*u[0])
            rings=[]
            for center,radius in ((a,r),(b,r if end_r is None else end_r)):
                rings.append([tuple(center[j]+radius*(math.cos(t)*u[j]+math.sin(t)*v[j]) for j in range(3))
                              for t in (2*math.pi*i/segments for i in range(segments))])
            for i in range(segments):
                k=(i+1)%segments;polygon((rings[0][i],rings[0][k],rings[1][k],rings[1][i]),colour)
            polygon(tuple(reversed(rings[0])),colour);polygon(rings[1],colour)
            if solid:proxy(name,rings[0]+rings[1])

        def windows(x,y,z,n=5):
            for i in range(n):box('window',(x+i*.32,y,z),(.22,.035,.25),glass)

        if kind=='wind_turbine':
            tube('foundation',(0,0,-6),(0,0,.65),.54,yellow,True)
            tube('tower',(0,0,.65),(0,0,7.2),.28,white,True,end_r=.16)
            box('nacelle',(0,.12,7.25),(.62,.95,.42),white,True)
            tube('hub',(0,-.48,7.25),(0,-.78,7.25),.19,white,True)
            for angle in (0,2*math.pi/3,4*math.pi/3):
                s,c=math.sin(angle),math.cos(angle)
                points=[]
                for y in (-.77,-.69):
                    points.extend([(x*c+z*s,y,7.25-x*s+z*c) for x,z in
                                   ((-.10,.14),(.11,.14),(.18,.85),(.025,2.30),(-.035,2.30),(-.13,.75))])
                polygon(tuple(reversed(points[:6])),white);polygon(points[6:],white)
                for i in range(6):polygon((points[i],points[(i+1)%6],points[(i+1)%6+6],points[i+6]),white)
                proxy('blade_'+str(round(angle,2)),points)
            tube('service_ring',(0,0,.66),(0,0,.72),.72,steel)
            for z in (1.,2.,3.,4.,5.,6.):box('ladder',(0,-.30,z),(.24,.07,.035),steel)

        elif kind=='production_platform':
            for x in (-2.2,2.2):
                for y in (-1.8,1.8):tube('leg',(x,y,-6),(x,y,2.1),.27,steel,True)
            for y in (-1.8,1.8):
                tube('brace',(-2.2,y,-4),(2.2,y,.7),.065,steel,True)
                tube('brace',(2.2,y,-4),(-2.2,y,.7),.065,steel,True)
            box('deck',(0,0,2.),(5.6,4.8,.35),steel,True)
            box('accommodation',(.95,-.6,2.8),(2.45,1.65,1.25),white,True)
            box('roof',(.95,-.6,3.48),(2.6,1.8,.13),blue)
            windows(.05,-1.445,2.85,6)
            tube('helipad',(-1.25,.60,2.22),(-1.25,.60,2.30),1.05,yellow)
            tube('helipad_surface',(-1.25,.60,2.30),(-1.25,.60,2.32),.94,blue)
            for dx in (-.24,.24):box('H',(-1.25+dx,.6,2.34),(.07,.62,.025),white)
            box('H',(-1.25,.6,2.34),(.48,.075,.025),white)
            for x in (-2.7,2.7):
                for z in (2.65,2.95):tube('rail',(x,-2.3,z),(x,2.3,z),.035,yellow)
                for y in (-2.3,-1.2,0,1.2,2.3):tube('rail',(x,y,2.18),(x,y,2.95),.028,yellow)
            for y in (-2.3,2.3):
                for z in (2.65,2.95):tube('rail',(-2.7,y,z),(2.7,y,z),.035,yellow)
            tube('crane_mast',(1.95,1.4,2.2),(1.95,1.4,4.15),.18,orange,True)
            tube('crane_boom',(1.95,1.4,4.15),(3.25,.2,5.55),.09,orange,True)
            tube('crane_cable',(3.25,.2,5.55),(3.25,.2,2.55),.012,steel)
            for x,y in ((.4,-.1),(1.3,-.1)):
                tube('antenna',(x,y,3.5),(x,y,4.9),.035,white)
                tube('antenna',(x-.22,y,4.45),(x+.22,y,4.45),.025,white)
            for z in (0,.3,.6,.9,1.2,1.5,1.8):box('stairs',(-2.5,-1.3,z),(.55,.30,.045),yellow)

        elif kind=='seabed_pipeline':
            points=[tuple(p) for p in spec['points']]
            for n,(a,b) in enumerate(zip(points,points[1:])):
                tube('segment_'+str(n),a,b,.16,steel,True,segments=16)
                distance=math.dist(a,b)
                for k in range(1,int(distance)):
                    t=k/distance;p=tuple(a[j]+t*(b[j]-a[j]) for j in range(3))
                    q=tuple(p[j]+.05*(b[j]-a[j])/distance for j in range(3))
                    tube('joint',p,q,.19,stone)
            for n,p in enumerate(points[1:]):
                tube('valve_'+str(n),(p[0],p[1],p[2]-.01),(p[0],p[1],p[2]+.42),.20,yellow,True)
                tube('valve_wheel',(p[0],p[1],p[2]+.42),(p[0],p[1],p[2]+.45),.29,yellow)

        elif kind=='command_vessel':
            outline=[(-4.8,-.95),(-4.8,.95),(1.9,1.12),(3.8,.42),(4.1,0),(3.8,-.42),(1.9,-1.12)]
            lower=[(x*.96,y*.72,-.62) for x,y in outline];upper=[(x,y,.5) for x,y in outline]
            polygon(lower,blue);polygon(tuple(reversed(upper)),white)
            for i in range(len(outline)):j=(i+1)%len(outline);polygon((lower[i],upper[i],upper[j],lower[j]),blue)
            box('core',(-.35,0,.95),(8.9,2.25,3.8),blue,True)
            # The conservative core above is collision metadata only; its
            # faces are replaced visually by the detailed hull/deck below.
            parts[blue]=parts[blue][:-12]
            box('cabin',(-1.,0,1.03),(3.4,1.4,1.1),white)
            box('bridge',(-.6,0,1.79),(2.35,1.1,.6),white)
            box('roof',(-.6,0,2.12),(2.55,1.28,.12),blue)
            windows(-1.7,-.565,1.83,7);windows(-2.4,-.715,1.1,8)
            tube('mast',(-1.,0,2.18),(-1.,0,2.7),.04,white)
            box('radar',(-1.,0,2.76),(.65,.14,.12),white)
            for y in (-.94,.94):
                tube('deck_rail',(-4.4,y,.95),(2.1,y,.95),.025,white)
                for x in (-4.4,-3.,-1.5,0,1.5):tube('rail_post',(x,y,.52),(x,y,.95),.022,white)
            for x in (-3.3,2.3):tube('bollard',(x,0,.52),(x,0,.69),.09,steel)

        elif kind=='shore_port':
            box('land',(-8.,0.,-2.25),(16.,44.,7.5),stone,True)
            parts[stone]=parts[stone][:-12]
            def elevation(x,y):
                return 1.5+max(0.,-x-11.)*.42+.12*math.sin(x*.8+y*.5)*max(0.,min(1.,-x-10.))
            for ix in range(8):
                for iy in range(11):
                    x=-15.+ix*2;y=-20.+iy*4
                    colour=(.30+.015*((ix+iy)%4),.39+.02*((2*ix+iy)%3),.28,1.)
                    polygon([(a,b,elevation(a,b)) for a,b in
                             ((x-1,y-2),(x+1,y-2),(x+1,y+2),(x-1,y+2))],colour)
            for a,b in (((0,-22),(0,22)),((-16,22),(0,22)),((0,-22),(-16,-22))):
                polygon(((a[0],a[1],-6),(b[0],b[1],-6),
                         (b[0],b[1],elevation(*b)),(a[0],a[1],elevation(*a))),stone)
            box('road',(-6.4,0,1.54),(2.0,43.,.025),(.17,.20,.20,1.))
            for y in range(-20,22,3):box('lane',(-6.4,y,1.56),(.06,1.4,.025),white)
            box('command_center',(-8.8,8.,3.0),(5.6,3.9,2.9),white)
            box('command_roof',(-8.8,8.,4.48),(5.8,4.1,.10),steel)
            for z in (2.25,3.25):windows(-11.2,6.04,z,15)
            tube('antenna',(-8.7,8,4.5),(-8.7,8,6.5),.04,steel)
            tube('antenna',(-9.1,8,6.0),(-8.3,8,6.0),.025,steel)
            tube('lighthouse',(-2.0,18.,1.55),(-2.0,18.,4.65),.40,white)
            tube('lighthouse_band',(-2.0,18.,3.8),(-2.0,18.,4.05),.415,orange)
            tube('lighthouse_lantern',(-2.0,18.,4.65),(-2.0,18.,5.1),.31,glass)
            tube('lighthouse_roof',(-2.0,18.,5.1),(-2.0,18.,5.40),.5,steel,end_r=.01)
            for x,y in ((-3.,13),(-4.2,13),(-3.,-9),(-4.2,-9)):
                box('container',(x,y,2.05),(.95,2.35,1.0),blue if x==-3 else orange)
                for k in range(8):box('corrugation',(x-.49,y-1.+k*.28,2.05),(.015,.035,.96),steel)
            tube('crane',(-2.8,9,1.6),(-2.8,9,4.0),.13,yellow)
            tube('crane',(-2.8,9,4.0),(.2,9,5.0),.08,yellow)
            for ix in range(5):
                for iy in range(8):
                    x=-15.+ix*1.4;y=-18.+iy*5+(ix%2)*.6
                    h=1.0+.25*((ix+iy)%4)
                    base=elevation(x,y)+.04
                    tube('tree_trunk',(x,y,base),(x,y,base+.35),.065,wood)
                    tube('tree_crown',(x,y,base+.25),(x,y,base+.25+h),.5,(.12,.29+.025*(iy%3),.13,1.),segments=9,end_r=.015)

        elif kind=='harbor_dock':
            length=float(spec.get('length_m',13.))
            box('deck',(0,0,.7),(length,1.1,.35),stone,True)
            box('shore_connection',(-length/2-.5,0,.7),(1.,1.1,.35),stone,True)
            for x in (-length/2+.4,-2,2,length/2-.4):
                for y in (-.38,.38):tube('pile',(x,y,-6),(x,y,.65),.12,wood,True)
            for x in range(-int(length/2),int(length/2)+1):
                box('deck_joint',(x,0,.89),(.022,1.05,.02),steel)
            for x in (-length/2+.6,length/2-.6):tube('bollard',(x,0,.9),(x,0,1.08),.08,steel)

        elif kind=='aav':
            box('fuselage',(0,0,0),(.19,.10,.085),white)
            box('battery',(-.025,0,.053),(.095,.063,.026),blue)
            for x in (-.12,.12):
                for y in (-.12,.12):
                    tube('arm',(0,0,0),(x,y,.005),.012,steel)
                    tube('motor',(x,y,0),(x,y,.045),.022,white)
                    tube('rotor',(x,y,.047),(x,y,.052),.044,steel,segments=18)
            for y in (-.055,.055):tube('skid',(-.08,y,-.06),(.09,y,-.06),.007,steel)
        else:raise ValueError('unknown display model: '+kind)

        root=ET.Element('COLLADA',xmlns='http://www.collada.org/2005/11/COLLADASchema',version='1.4.1')
        asset=ET.SubElement(root,'asset');ET.SubElement(asset,'unit',name='meter',meter='1');ET.SubElement(asset,'up_axis').text='Z_UP'
        effects=ET.SubElement(root,'library_effects');materials=ET.SubElement(root,'library_materials')
        geometries=ET.SubElement(root,'library_geometries');scenes=ET.SubElement(root,'library_visual_scenes')
        visual=ET.SubElement(scenes,'visual_scene',id='scene');all_vertices=[];count=0
        for index,(colour,triangles) in enumerate(parts.items()):
            if not triangles:continue
            key='part'+str(index);effect=ET.SubElement(effects,'effect',id=key+'fx')
            profile=ET.SubElement(effect,'profile_COMMON');phong=ET.SubElement(ET.SubElement(profile,'technique',sid='common'),'phong')
            ET.SubElement(ET.SubElement(phong,'diffuse'),'color').text=' '.join(map(str,colour))
            ET.SubElement(ET.SubElement(phong,'ambient'),'color').text=' '.join(map(str,colour))
            ET.SubElement(ET.SubElement(phong,'emission'),'color').text=' '.join(str(v*.16) for v in colour[:3])+' 1'
            material=ET.SubElement(materials,'material',id=key+'mat');ET.SubElement(material,'instance_effect',url='#'+key+'fx')
            mesh=ET.SubElement(ET.SubElement(geometries,'geometry',id=key),'mesh')
            vertices=[p for row in triangles for p in row[:3]];normals=[row[3] for row in triangles for _ in range(3)]
            all_vertices+=vertices;count+=len(triangles)
            for name,data in (('pos',vertices),('normal',normals)):
                source=ET.SubElement(mesh,'source',id=key+name)
                ET.SubElement(source,'float_array',id=key+name+'array',count=str(len(data)*3)).text=' '.join(format(v,'.7g') for p in data for v in p)
                accessor=ET.SubElement(ET.SubElement(source,'technique_common'),'accessor',source='#'+key+name+'array',count=str(len(data)),stride='3')
                for component in 'XYZ':ET.SubElement(accessor,'param',name=component,type='float')
            vertex=ET.SubElement(mesh,'vertices',id=key+'v');ET.SubElement(vertex,'input',semantic='POSITION',source='#'+key+'pos')
            tri=ET.SubElement(mesh,'triangles',count=str(len(triangles)),material=key+'symbol')
            ET.SubElement(tri,'input',semantic='VERTEX',source='#'+key+'v',offset='0')
            ET.SubElement(tri,'input',semantic='NORMAL',source='#'+key+'normal',offset='1')
            ET.SubElement(tri,'p').text=' '.join(str(i) for i in range(len(vertices)) for _ in range(2))
            node=ET.SubElement(visual,'node',id=key+'node');instance=ET.SubElement(node,'instance_geometry',url='#'+key)
            ET.SubElement(ET.SubElement(ET.SubElement(instance,'bind_material'),'technique_common'),'instance_material',symbol=key+'symbol',target='#'+key+'mat')
        ET.SubElement(ET.SubElement(root,'scene'),'instance_visual_scene',url='#scene')
        filename=spec['id']+'.dae';ET.ElementTree(root).write(output/filename,encoding='utf-8',xml_declaration=True)
        offset=tuple(spec.get('position',(0,0,0)))
        for n,(name,center,size) in enumerate(local_boxes):
            proxies.append(dict(id=spec['id']+'_'+name+'_'+str(n),kind='SOLID',
                center=[offset[j]+center[j] for j in range(3)],size=size,appearance='proxy',facility=spec['id']))
        models.append(dict(id=spec['id'],model=kind,mesh=filename,position=list(offset)))
        records[filename]=dict(source='project procedural geometry; user-approved scene reference',
            triangles=count,bounds=[[min(p[j] for p in all_vertices) for j in range(3)],
                                   [max(p[j] for p in all_vertices) for j in range(3)]],
            proxy_count=len(local_boxes))
    return models,proxies,records


def convert(source, destination, texture=None):
    vertices, normals, uv, triangles = [], [], [], []
    for line in source.read_text().splitlines():
        fields=line.split()
        if not fields:continue
        if fields[0]=='v':vertices.append(tuple(float(v) for v in fields[1:4]))
        elif fields[0]=='vn':normals.append(tuple(float(v) for v in fields[1:4]))
        elif fields[0]=='vt':uv.append(tuple(float(v) for v in fields[1:3]))
        elif fields[0]=='f':
            face=[]
            for word in fields[1:]:
                indices=word.split('/')
                def index(value, size):
                    number=int(value);return number-1 if number>0 else size+number
                face.append((index(indices[0],len(vertices)),
                             index(indices[2],len(normals)),
                             index(indices[1],len(uv)) if len(indices)>1 and indices[1] else 0))
            for i in range(1,len(face)-1):triangles.extend([face[0],face[i],face[i+1]])
    if not uv:uv=[(0.,0.)]
    root=ET.Element('COLLADA',xmlns='http://www.collada.org/2005/11/COLLADASchema',version='1.4.1')
    asset=ET.SubElement(root,'asset');ET.SubElement(asset,'unit',name='meter',meter='1');ET.SubElement(asset,'up_axis').text='Z_UP'
    if texture:
        imgs=ET.SubElement(root,'library_images');img=ET.SubElement(imgs,'image',id='texture')
        ET.SubElement(img,'init_from').text=texture
    effects=ET.SubElement(root,'library_effects');effect=ET.SubElement(effects,'effect',id='surface-effect')
    profile=ET.SubElement(effect,'profile_COMMON')
    if texture:
        param=ET.SubElement(profile,'newparam',sid='surface');surface=ET.SubElement(param,'surface',type='2D');ET.SubElement(surface,'init_from').text='texture'
        param=ET.SubElement(profile,'newparam',sid='sampler');sampler=ET.SubElement(param,'sampler2D');ET.SubElement(sampler,'source').text='surface'
    technique=ET.SubElement(profile,'technique',sid='common');phong=ET.SubElement(technique,'phong')
    diffuse=ET.SubElement(phong,'diffuse')
    if texture:ET.SubElement(diffuse,'texture',texture='sampler',texcoord='UVSET0')
    else:ET.SubElement(diffuse,'color').text='0.38 0.40 0.42 1'
    mats=ET.SubElement(root,'library_materials');mat=ET.SubElement(mats,'material',id='surface-material');ET.SubElement(mat,'instance_effect',url='#surface-effect')
    geometries=ET.SubElement(root,'library_geometries');geometry=ET.SubElement(geometries,'geometry',id='mesh');mesh=ET.SubElement(geometry,'mesh')
    for name,rows,components in [('position',vertices,'XYZ'),('normal',normals,'XYZ'),('uv',uv,'ST')]:
        element=ET.SubElement(mesh,'source',id=name)
        ET.SubElement(element,'float_array',id=name+'-array',count=str(len(rows)*len(components))).text=' '.join(str(v) for row in rows for v in row)
        common=ET.SubElement(element,'technique_common');accessor=ET.SubElement(common,'accessor',source='#'+name+'-array',count=str(len(rows)),stride=str(len(components)))
        for component in components:ET.SubElement(accessor,'param',name=component,type='float')
    v=ET.SubElement(mesh,'vertices',id='vertices');ET.SubElement(v,'input',semantic='POSITION',source='#position')
    tri=ET.SubElement(mesh,'triangles',count=str(len(triangles)//3),material='surface-symbol')
    ET.SubElement(tri,'input',semantic='VERTEX',source='#vertices',offset='0')
    ET.SubElement(tri,'input',semantic='NORMAL',source='#normal',offset='1')
    ET.SubElement(tri,'input',semantic='TEXCOORD',source='#uv',offset='2',set='0')
    ET.SubElement(tri,'p').text=' '.join(str(v) for row in triangles for v in row)
    scenes=ET.SubElement(root,'library_visual_scenes');scene=ET.SubElement(scenes,'visual_scene',id='scene');node=ET.SubElement(scene,'node',id='model')
    instance=ET.SubElement(node,'instance_geometry',url='#mesh');bind=ET.SubElement(instance,'bind_material');common=ET.SubElement(bind,'technique_common')
    material=ET.SubElement(common,'instance_material',symbol='surface-symbol',target='#surface-material')
    ET.SubElement(material,'bind_vertex_input',semantic='UVSET0',input_semantic='TEXCOORD',input_set='0')
    scene=ET.SubElement(root,'scene');ET.SubElement(scene,'instance_visual_scene',url='#scene')
    ET.ElementTree(root).write(destination,encoding='utf-8',xml_declaration=True)
    return {'vertices':len(vertices),'triangles':len(triangles)//3,
            'bounds':[[min(p[i] for p in vertices) for i in range(3)],
                      [max(p[i] for p in vertices) for i in range(3)]]}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path)
    parser.add_argument('--scene',type=Path);args=parser.parse_args()
    root=Path(__file__).resolve().parents[1];source=root/'upstream/Stonefish/Tests/Data'
    args.output.mkdir(parents=True,exist_ok=True)
    metadata={'source':'https://github.com/patrykcieslak/Stonefish','license':'GPL-3.0-or-later',
              'role':'visual assets only; no Stonefish runtime','files':{}}
    for name,target,texture in [('aquadelmo.obj','mother_ship.dae','mother_ship.png'),('icosphere.obj','rock.dae',None)]:
        record=convert(source/name,args.output/target,texture)
        record.update(source_path=str((source/name).relative_to(root)),source_sha256=hashlib.sha256((source/name).read_bytes()).hexdigest(),
                      generated_sha256=hashlib.sha256((args.output/target).read_bytes()).hexdigest())
        metadata['files'][target]=record
    texture=source/'aquadelmo_tex.png'
    shutil.copyfile(texture,args.output/'mother_ship.png')
    metadata['texture_sha256']=hashlib.sha256(texture.read_bytes()).hexdigest()
    shutil.copyfile(root/'upstream/Stonefish/COPYING.txt',args.output/'COPYING.txt')
    if args.scene:
        import yaml
        config=yaml.safe_load(args.scene.read_text());scene=config['scene']
        models,proxies,records=world_models(args.output,scene)
        scene['visual_models']=models;metadata['files'].update(records)
        if models:
            has_vessel=any(m['model']=='command_vessel' for m in models)
            scene['objects']=[o for o in scene['objects'] if not o.get('facility') and
                              not (has_vessel and o['id']=='mother_ship_hull')]+proxies
            for obj in scene['objects']:
                if obj['id']=='shore':obj['appearance']='proxy'
            (args.output/'expanded-scene.yaml').write_text(yaml.safe_dump(config,allow_unicode=True,sort_keys=False))
        metadata['license_note']='GPL applies to the original Stonefish files; procedural facility files are project-generated geometry.'
    (args.output/'sources.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print('已复用本地船模与岩石网格，资源记录：',args.output/'sources.json')


if __name__=='__main__':main()
