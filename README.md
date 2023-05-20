# LocalRemesher

Blender addon for remeshing selected area with awarness of surrounding geometry.

## Installation

- Download the latest release
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
