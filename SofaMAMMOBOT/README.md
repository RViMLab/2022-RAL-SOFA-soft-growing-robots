# SofaMAMMOBOT plugin

This plugin contains different SOFA functions and classes to add an steerable catheter or pressure based eversion robot into your custom SOFA scenes. It offers the possibility to control each of the models on the scene. All the functionalities are built using SofaPython3, so it is required to use this plugin, you can follow the compilation instruction from their page [SofaPython3 how to build](https://sofapython3.readthedocs.io/en/latest/menu/Compilation.html).

## Starting SofaMAMMOBOT
Clone the github repository onto the sofa plugins folder. And add the following line to *applications/plugins/CMakeLists.txt*.

`sofa_add_subdirectory(plugin SofaMAMMOBOT SofaMAMMOBOT)`

Build SOFA following the instructions in [SOFA-Framework](https://www.sofa-framework.org/community/doc/). Activate the build of SOFAEversion on the CMake GUI.

Once built, check that the scripts were created under the SOFA libraries site-packages (*lib/python3/site-packages/SOFAEversion*) and add this directory to your Python path to use the functions.

Try it: ```python import SofaMAMMOBOT```

## How to use the plugin
The plugin contains all built nodes modelling the MAMMOBOT system, only needs a simple import function to access it. 

```python3
import Sofa
import Sofa.Core as SC
from SofaMAMMOBOT.nodes import catheter

parent = SC.Node("root")
#Catheter node with default values
catheter.CatheterNode(parent)
```

Some parameters of the catheter and the sheath may be updated by the user, but it can cause the scene to diverge. Default data for the catheter and the sheath are based on experimental analysis. Useful examples can be found in *examples* folder.