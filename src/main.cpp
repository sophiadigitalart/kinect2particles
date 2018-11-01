#pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"") // Hide console window

#include "ofMain.h"
#include "ofApp.h"

//========================================================================
int main( ){
    // this example uses compute shaders which are only supported since
    // openGL 4.3
	//ofGLWindowSettings settings;
	ofGLFWWindowSettings settings;
	settings.setGLVersion(4,3);
	settings.windowMode = OF_FULLSCREEN;
	//settings.resizable = false;
	ofCreateWindow(settings);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new ofApp());

}
