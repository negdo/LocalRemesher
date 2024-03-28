# LocalRemesher

Blender addon for remeshing selected area with awarness of surrounding geometry.

I implemented two methods based on my own ideas for solving the problem of local remeshing and topology. Both methods can produce satisfyable results, but also fail in some cases. Robustness of the algorithms is not yet on the level to be good for an average user.

## Installation

- Download the LocalRemesher.zip from the latest [release](https://github.com/negdo/LocalRemesher/releases)
- Open Blender
- Go to `Edit > Preferences > Add-ons > Install...`
- Select the downloaded file
- Enable the addon

## Usage

- Select an object and enter edit mode
- Select an area you want to remesh
- Navigate to right panel in 3D view, find `Local Remesher` tab
- Click one of the buttons to remesh the selected area

## Tips

The addon is not yet perfect, so here are some tips to get better results:
- Try to select a single closed area of convex shape
- Surrounding geometry should contain good topology
- If the result is not good, try selecting a smaller or larger area
- Particle-based method is better for smaller areas
- Grid-based method is better for a little larger areas

## Algorithms

### Grid-based method
Works by initializing points at the intersections of the existing edge vectors. Created vertices get connected to their neighbours and then geometry is refined and filled with faces.

<img src="https://github.com/negdo/LocalRemesher/assets/18052453/8c8f6755-0fe6-4a93-a762-ad4168c29380" width="600">
<img src="https://github.com/negdo/LocalRemesher/assets/18052453/e0e8f7e2-740d-4303-83f0-783b531313a5" width="600">
<img src="https://github.com/negdo/LocalRemesher/assets/18052453/0c0229cd-4842-46f9-9c47-6bb030c4294e" width="600">

### Particle-based method

This method samples random points on the surface of original mesh. In each iteration new best particle is chosen and added to mesh.

