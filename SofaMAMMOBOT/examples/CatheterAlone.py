import Sofa
import Sofa.Core as SC
from SofaMAMMOBOT.sceneGenerate import MainHeader, addController
from SofaMAMMOBOT.nodes.catheter import CatheterNode
from SofaMAMMOBOT.controllers.catheter_controller import CatheterController

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

    MainHeader(parent)

    #Introduce your custom values here for the catheter
    CatheterNode(parent, name="catheter", gridSize=[1, 4, 152], collgridSize=[1, 1, 500], scale = [1, 1, 1], 
    translation=[0, 0, 0], rotation=[0, 0, 0], stiffness=8e10, damping=4e3, totalMass=3.14e-3, createTendon=True,
    tendonValues={"mass": 3.14e-4, "linesStiffness": 2e9, "linesDamping": 40, "restScale": 1.0})

    #Catheter controller
    addController(parent, CatheterController)

    return parent

#Run the script on the Python IDLE
if __name__ == "__main__":
    main()
