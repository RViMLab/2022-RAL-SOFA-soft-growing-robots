import Sofa
import Sofa.Core as SC
from SofaMAMMOBOT.sceneGenerate import MainHeader, addController, CylinderNavigation
from SofaMAMMOBOT.nodes.catheter import CatheterNode
from SofaMAMMOBOT.nodes.sheath import SheathNode
from SofaMAMMOBOT.controllers.navigation_controller import NavigationController

def main():
    import SofaRuntime
    import Sofa.Gui

    SofaRuntime.importPlugin("SofaOpenglVisual")
    SofaRuntime.importPlugin("SofaImplicitOdeSolver")
    SofaRuntime.importPlugin("SofaLoader")
    SofaRuntime.importPlugin("SofaRigid")

    root = Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)
   
    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1920, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()

#Necessary function to run the scene using runSofa
def createScene(parent):

    #Contains the needed plugins, collision pipeline and visual style of the scene
    MainHeader(parent)

    #Introduce your custom values here for the catheter
    CatheterNode(parent, name="catheter", gridSize=[1, 4, 152], collgridSize=[1, 1, 500], scale = [1, 1, 1], 
    translation=[0, 0, 0], rotation=[0, 0, 0], stiffness=8e10, damping=4e3, totalMass=3.14e-3, createTendon=True,
    tendonValues={"mass": 3.14e-4, "linesStiffness": 2e9, "linesDamping": 40, "restScale": 1.0})

    #Introduce your custom values here for the sheath
    SheathNode(parent, name="Sheath", gridSize=[1, 1, 25], collgridSize=[1, 1, 62], scale = [1, 1, 1], 
    translation=[1, -3.3, 20], rotation=[0, 0, 45], stiffness=3e8, damping=4.0, totalMass=3.14e-3, 
    restScale=1.0, pressure=-7e4)

    #Introduce the cylindrical environment with custom number of cylinders and positions
    n_cylinders = 6
    positions = [[5, -5, 170], [-2, -5, 195], [-18, -5, 195], [-18, -5, 215], [12, -5, 220], [0, -7, 205], [0, 6, 205]]
    radii = [5, 5, 5, 5, 100, 100]
    lengths = [10, 10, 10, 10, 1, 1]
    CylinderNavigation(parent, gridSize=[5, 15, 5], axis=[0,1,0], n_cylinders=n_cylinders,
    position=positions, radii=radii, lengths=lengths)

    #Sheath controller
    addController(parent, NavigationController)

    return parent

#Run the script on the Python IDLE
if __name__ == "__main__":
    main()