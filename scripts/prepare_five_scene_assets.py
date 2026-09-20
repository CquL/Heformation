#!/usr/bin/env python3
"""Convert already-vendored Stonefish display assets to RViz COLLADA.

Offline format conversion only. No physics, controller or new online node.
Source OBJ, texture, GPL license and source hashes accompany the generated mesh.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


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
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);args=parser.parse_args()
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
    (args.output/'sources.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print('已复用本地船模与岩石网格，资源记录：',args.output/'sources.json')


if __name__=='__main__':main()
