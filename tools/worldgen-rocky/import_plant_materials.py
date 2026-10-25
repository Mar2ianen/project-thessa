#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Import pinned CC0 optical maps; source images are offline-only inputs."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
from PIL import Image

source=Path(sys.argv[1]); destination=Path(sys.argv[2]);destination.mkdir(parents=True,exist_ok=True)
SOURCE_SHA256 = {'bark_willow_diff_1k.png': '002a5c88826d34c96f32a5a8f72c372fb172cc1129bd7fa2b70c0e087cb2e4b7', 'bark_willow_nor_gl_1k.png': 'cfcba7785407ba0b3fa3cba115ad3dbc8c1c5f62ee0d4b0c63dbc0e550109fc1', 'bark_willow_rough_1k.png': '14d001fc51e10f0ae37e5eafa71d39ff909d4fd8de476a65889757b767dbe3b0', 'fern_02_diff_2k.png': '25168368e55d56e1d4eb97aaf72b1ea7c59b61aa75d64ebca86150a34762a583', 'fern_02_nor_gl_2k.png': '0f66d392641b547bc4e5a2254f93177db2b28dadbe03c8eaac92ff8e8cb5def9', 'fern_02_rough_2k.png': 'ab0f177eb54647de5bd6b889b9be7a3e7d427b1a9bd5135412eb8a6c0e1a586f'}
records=[]
def read(asset,role):
    files=list((source/asset).glob('*_'+role+'_*.png'))
    if len(files) != 1: raise ValueError(f'Expected one {asset}/{role} source')
    path=files[0];raw=path.read_bytes(); digest=hashlib.sha256(raw).hexdigest()
    if SOURCE_SHA256.get(path.name) != digest: raise ValueError(f'Unpinned source: {path.name}')
    records.append(dict(source=path.name,sha256=digest))
    value=np.asarray(Image.open(path).convert('RGB'),dtype=np.float64)/255
    return value

def linear(a): return np.where(a<=.04045,a/12.92,((a+.055)/1.055)**2.4)
def srgb(a): return np.where(a<=.0031308,12.92*a,1.055*a**(1/2.4)-.055)
def write(name,rgb):
    data=np.empty((*rgb.shape[:2],4),np.uint8);data[:,:,:3]=np.rint(np.clip(rgb,0,1)*255).astype(np.uint8);data[:,:,3]=255
    Image.fromarray(data).save(destination/(name+'.png'),optimize=True)

def sample(a,x,y):
    ix=np.floor(x).astype(int);iy=np.floor(y).astype(int);fx=(x-ix)[...,None];fy=(y-iy)[...,None]
    assert ix.min()>=0 and iy.min()>=0 and ix.max()+1<a.shape[1] and iy.max()+1<a.shape[0]
    return (a[iy,ix]*(1-fx)+a[iy,ix+1]*fx)*(1-fy)+(a[iy+1,ix]*(1-fx)+a[iy+1,ix+1]*fx)*fy

for asset,tag in [('bark_willow','bark'),('fern_02','pinna')]:
    diffuse=linear(read(asset,'diff'));normal=read(asset,'nor_gl')*2-1;rough=read(asset,'rough')
    if tag=='bark':
        def average(a): return (a[0::2,0::2]+a[1::2,0::2]+a[0::2,1::2]+a[1::2,1::2])*.25
        diffuse,normal,rough=map(average,[diffuse,normal,rough])
        # Preserve the published 8-bit downsample stage before the authored
        # gain, so the existing bark asset remains exactly reproducible.
        diffuse=linear(np.rint(np.clip(srgb(diffuse),0,1)*255)/255)
        diffuse=np.clip(diffuse*np.array([2.4,3.6,5.2]),0,1)
    else:
        # One actual pinna, not the whole alpha-masked fern card. The source
        # padding is kept; runtime silhouette remains explicit mesh geometry.
        v,u=np.mgrid[0:48,0:128];u=(u+.5)/128;v=(v+.5)/48-.5
        x=323-105*u-10*v;y=1518+30*u-35*v
        diffuse,normal,rough=[sample(a,x,y) for a in [diffuse,normal,rough]]
        # Authored scattering albedo for the reference archetype, not calibrated
        # source photometry or a measured leaf spectrum. Apply in linear RGB.
        diffuse=np.clip(diffuse*np.array([2.6,2.5,1.8]),0,1)
        tangent=np.array([-105.,30.]);tangent/=np.linalg.norm(tangent)
        bitangent=np.array([-30.,-105.]);bitangent/=np.linalg.norm(bitangent)
        xy=normal[:,:,:2].copy();normal[:,:,0]=xy@tangent;normal[:,:,1]=xy@bitangent
    normal/=np.linalg.norm(normal,axis=2,keepdims=True)
    write(tag+'-base',srgb(diffuse));write(tag+'-normal',normal*.5+.5)
    mr=np.ones_like(rough);mr[:,:,1]=rough[:,:,0];mr[:,:,2]=0;write(tag+'-mr',mr)
manifest={'sources':records,'derived':[],'leaf_affine_2k_pixels':{'origin':[323,1518],'length_axis':[-105,30],'width_axis':[-10,-35]},'bark_width_m':1.0,'leaf_is_authored_scaled_archetype':True,'leaf_authored_linear_albedo_gain':[2.6,2.5,1.8],'bark_authored_linear_albedo_gain':[2.4,3.6,5.2],'bark_gain_input':'quantized_srgb8_after_linear_downsample'}
for path in sorted(destination.glob('*.png')):manifest['derived'].append(dict(path=path.name,bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
(destination/'provenance.json').write_text(json.dumps(manifest,indent=2)+'\n')
