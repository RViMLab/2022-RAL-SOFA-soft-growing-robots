import Sofa
import Sofa.Core as SC
from SofaMAMMOBOT.sceneGenerate import MainHeader, addController
from SofaMAMMOBOT.nodes.sheath import SheathNode
from SofaMAMMOBOT.controllers.sheath_controller import SheathController

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

    #Introduce your custom values here for the sheath
    SheathNode(parent, name="Sheath", gridSize=[1, 1, 25], collgridSize=[1, 1, 62], scale = [1, 1, 1], 
    translation=[1, -3.3, 20], rotation=[0, 0, 45], stiffness=3e8, damping=4.0, totalMass=3.14e-3, 
    restScale=1.0, pressure=-7e4)

    #Sheath controller
    addController(parent, SheathController)

    return parent

#Run the script on the Python IDLE
if __name__ == "__main__":
    main()