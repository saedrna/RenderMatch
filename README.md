# RenderMatch

Implementation of our paper ["Leveraging Photogrammetric Mesh Models for Aerial-Ground Feature Point Matching Toward Integrated 3D Reconstruction"](https://vrlab.org.cn/~hanhu/projects/meshmatch/) submitted to ISPRS Journal of Photogrammetry and Remote Sensing

## Dependencies

The code is extracted from our private code repository, currently many libraries are dependent for some legacy issues. To install the dependencies, we provide anaconda environment file. Using the environment file ```environment.Linux.yml``` and ```environment.msvc.yml``` for building with Linux gcc and VS 2017 (we only test it with MSVC 2017), respectively.

The environment is built using these [recipies](https://github.com/saedrna/AnacondaRecipies), and the binaries are deployed in [anaconda](http://anaconda.org/saedrna/).

- Linux: create environment with ```conda env create -n h2o -f environment.Linux.yml``` to create an environment named ```h2o``` (other names are also OK).
- MSVC 2017: create environment with ```conda env create -n h2o -f environment.msvc.yml```.

- For windows, we also provide a cmake tool chain for easy deployment, run

```bash
conda activate h2o
cd toolchains
robocopy ./ %CONDA_PREFIX% h2o.cmake
robocopy ./ %CONDA_PREFIX%/Library/plugins applocal.ps1
robocopy ./ %CONDA_PREFIX%/Library/plugins qtdeploy.ps1
robocopy ./ %CONDA_PREFIX% applocal.ps1
```

## Building

- activate the environment ```conda activate h2o```
- run the building script ```./build.sh``` for Linux and ```./build.bat``` for MSVC.

## Running

We have made several routines and currently we only test it on MSVC, including

- ```FeatureExtract```: Extract SIFT keypoints and descriptors for the ground images, a AT file in the [BlockExchange Format](https://docs.bentley.com/LiveContent/web/ContextCapture%20Help-v10/en/GUID-59E6CC36-F349-4DE0-A563-FFC47296A624.html) is needed.
- ```RenderMesh```: Render the mesh onto the ground views, color, depth, normal and XYZ maps can be generated.
- ```RenderMeshMatch```: Render the image, match with ground image, propagate to aerial views, refine the match and export the results.
- ```TileImage```: tile original image into blocked TIFF format. This is used for fast loading. We have optimized the file loading and encoding using multi-thread.
- ```UndistortImage```: This is currently not used.

A common routine to do the aerial-ground matching is like

```bash
TileImage -i Aerial.xml -o ./AerialTiled
FeatureExtract -i ./AerialTiled/Aerial.json
RenderMeshMatch -a ./AerialTiled/Aerial.json -g Ground.xml -m model.osgb -c config.json
```

## DATASET

We release all the images, initial AT (Aerial Triangulation) Files and the Tiled 3D models.

- Dortmund (3.1 GB): [Google Drive](https://drive.google.com/file/d/1xKHugRam0BDPZwoQMKwJExWzbRQLF9IP/view?usp=sharing), [BaiduYun Code: lukz](https://pan.baidu.com/s/1IjDVm_iVABUyOo4eN2vRBA)
- Zeche (3.2 GB): [Google Drive](https://drive.google.com/file/d/1HHrNpUFtmVPKFK6V4eNFSPBk1N-fUwAe/view?usp=sharing), [BaiduYun Code: 9x6t](https://pan.baidu.com/s/1t-0W_LoJ1rLIqp3ey2sWbA)
- SWJTU-BLD (2.7 GB): [Google Drive](https://drive.google.com/open?id=1KUIpckI9sTG_5RlUN64OnUS8Cq1HA3dK), [BaiduYun Code: e1p2](https://pan.baidu.com/s/14X1_toNLy2C1SYPtdewZiQ)
- SWJTU-LIB (2.0 GB): [Google Drive](https://drive.google.com/open?id=1gNDDLmTmdLfO699YUAe0BVkqdlYV3W5-), [BaiduYun Code: wqnj](https://pan.baidu.com/s/1hlNBm8d5G62IRnfK0omQ1w)
- SWJTU-RES (2.2 GB): [Google Drive](https://drive.google.com/open?id=1HH7M3qqwNhM-Ohhr8l37apXwYA8IpfkQ), [BaiduYun Code: z1fv](https://pan.baidu.com/s/1UnliVR6HjFu3edWURerReA)

## TODO List

There are several differences for the code and the paper.

- The normal is not implemented, because this requires a lot of tricks in the graphics pipeline and pre-processing of the models in tiled format.
- The occlusion detection is currently not ported.

## Licenses

- Data: [CC-BY-NC](https://creativecommons.org/licenses/) (Except for the images and derivatives of the Dortmund Datasets)
- Code: [MIT](https://opensource.org/licenses/MIT)
