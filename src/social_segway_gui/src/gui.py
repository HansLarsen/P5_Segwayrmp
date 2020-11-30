#!/usr/bin/env python2

## BEGIN_TUTORIAL
##
## Imports
## ^^^^^^^
##
## First we start with the standard ros Python import line:
import roslib; roslib.load_manifest('rviz_python_tutorial')

## Then load sys to get sys.argv.
import sys
import os
import rospkg
import rospy
from create_room.srv import Rvizcommand

## Next import all the Qt bindings into the current namespace, for
## convenience.  This uses the "python_qt_binding" package which hides
## differences between PyQt and PySide, and works if at least one of
## the two is installed.  The RViz Python bindings use
## python_qt_binding internally, so you should use it here as well.
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

## Finally import the RViz bindings themselves.
import rviz

## The MyViz class is the main container widget.
class MyViz( QWidget ):

    ## MyViz Constructor
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## Its constructor creates and configures all the component widgets:
    ## frame, thickness_slider, top_button, and side_button, and adds them
    ## to layouts.
    def __init__(self):
        QWidget.__init__(self)

        try:
            self.rvizcommand = rospy.ServiceProxy('rvizcommand', Rvizcommand)
            self.rvizcommand(a="delete_all")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        rospack = rospkg.RosPack()

        ## rviz.VisualizationFrame is the main container widget of the
        ## regular RViz application, with menus, a toolbar, a status
        ## bar, and many docked subpanels.  In this example, we
        ## disable everything so that the only thing visible is the 3D
        ## render window.
        self.frame = rviz.VisualizationFrame()

        ## The "splash path" is the full path of an image file which
        ## gets shown during loading.  Setting it to the empty string
        ## suppresses that behavior.
        self.frame.setSplashPath( "" )

        ## VisualizationFrame.initialize() must be called before
        ## VisualizationFrame.load().  In fact it must be called
        ## before most interactions with RViz classes because it
        ## instantiates and initializes the VisualizationManager,
        ## which is the central class of RViz.
        self.frame.initialize()

        ## The reader reads config file data into the config object.
        ## VisualizationFrame reads its data from the config object.
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, os.path.join(rospack.get_path('social_segway_gui'), "config.rviz" ))
        self.frame.load( config )

        ## You can also store any other application data you like in
        ## the config object.  Here we read the window title from the
        ## map key called "Title", which has been added by hand to the
        ## config file.
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

        ## Here we disable the menu bar (from the top), status bar
        ## (from the bottom), and the "hide-docks" buttons, which are
        ## the tall skinny buttons on the left and right sides of the
        ## main render window.
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )

        ## frame.getManager() returns the VisualizationManager
        ## instance, which is a very central class.  It has pointers
        ## to other manager objects and is generally required to make
        ## any changes in an rviz instance.
        self.manager = self.frame.getManager()

        ## Since the config file is part of the source code for this
        ## example, we know that the first display in the list is the
        ## grid we want to control.  Here we just save a reference to
        ## it for later.
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        
        ## Here we create the layout and other widgets in the usual Qt way.
        layout = QVBoxLayout()

        self.statusLabel = QLabel()
        self.statusLabel.setText("Input name, click current room name, add two points for a room, click save to save the current room, once all the rooms are in click save to xml.")
        layout.addWidget(self.statusLabel)

        layout.addWidget( self.frame )

        self.textBox = QLineEdit("Room Name here")
        layout.addWidget(self.textBox)
        
        h_layout = QHBoxLayout()
        
        top_button = QPushButton( "Set room name" )
        top_button.clicked.connect( self.onTopButtonClick )
        h_layout.addWidget( top_button )
        
        side_button = QPushButton( "Save current room" )
        side_button.clicked.connect( self.onSideButtonClick )
        h_layout.addWidget( side_button )

        publish_button = QPushButton( "Save rooms xml" )
        publish_button.clicked.connect( self.publishButtonClick )
        h_layout.addWidget( publish_button )
        
        layout.addLayout( h_layout )
        
        self.setLayout( layout )


    ## The view buttons just call switchToView() with the name of a saved view.
    def onTopButtonClick( self ):
        print(self.rvizcommand(a=self.textBox.text(),b=0))
        
        
    def onSideButtonClick( self ):
        print(self.rvizcommand(a="save"))

    def publishButtonClick( self ):
        print(self.rvizcommand(a="publish"))
        
    ## switchToView() works by looping over the views saved in the
    ## ViewManager and looking for one with a matching name.
    ##
    ## view_man.setCurrentFrom() takes the saved view
    ## instance and copies it to set the current view
    ## controller.
    def switchToView( self, view_name ):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )


def checkIfRosOK():
    if(rospy.is_shutdown()):
        sys.exit()

## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
##
## That is just about it.  All that's left is the standard Qt
## top-level application code: create a QApplication, instantiate our
## class, and start Qt's main event loop (app.exec_()).
if __name__ == '__main__':
    global app
    rospy.init_node('room_gui_node')
    app = QApplication( sys.argv )

    myviz = MyViz()
    myviz.resize( 1080, 1080 )
    myviz.show()

    timer = QTimer()
    timer.timeout.connect(checkIfRosOK)
    timer.start(1000)

    app.exec_()