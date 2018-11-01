#pragma once

#include "ofMain.h"
#include "ofBufferObject.h"
#include "ofxGui.h"
#include "ofxKinectForWindows2.h"
#include "ofxOsc.h"
#include "ofxSpout2Sender.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void dirAsColorChanged(bool & dirAsColor);

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void exit(); // added

		struct Particle{
			ofVec4f pos;
			ofVec4f vel;
			ofFloatColor color;
		};

		ofShader compute;
		vector<Particle> particles;
		ofBufferObject particlesBuffer, particlesBuffer2;
		GLuint vaoID;
		ofEasyCam camera;
		ofVbo vbo;
		ofVec3f atractor1, atractor2, atractor3;
		
		ofParameter<float> attractionCoeff, cohesionCoeff, repulsionCoeff;
		ofParameter<float> maxSpeed;
		ofParameter<float> attractor1Force, attractor2Force, attractor3Force;
		ofParameterGroup shaderUniforms;
		ofParameter<float> fps;
		ofParameter<bool> dirAsColor;
		ofxKFW2::Device kinect;
		ICoordinateMapper* coordinateMapper;

		void HostFieldChanged();

		// BG
		ofImage bgCB; // background checkerboard

		// offscreen buffers (frame buffer object)
		ofFbo fboDepth; // draw to for spout, setup at Kinect native 512x
		ofFbo fboColor; // draw to for spout, setup at 1080x

		// Spout obj
		ofxSpout2::Sender spout;
		string color_StreamName;
		string cutout_StreamName;
		string depth_StreamName;
		string keyed_StreamName;

		// OSC
		ofxOscSender oscSender;
		ofxOscReceiver oscReceiver;

		// custom functions DX
		void oscSendMsg(std::string message, std::string address);


		// GUI
		ofxPanel gui;

		ofxGuiGroup OSCgroup;
		ofxToggle jsonGrouped;
		// ofxInputField
		ofxIntField oscPort; // Output
		ofxIntField oscPortIn;
		ofxTextField HostField;

		ofxGuiGroup SPOUTgroup;
		ofxToggle spoutCutOut;
		ofxToggle spoutColor;
		ofxToggle spoutKeyed;
		ofxToggle spoutDepth;

		// added for coordmapping
		ofImage bodyIndexImg, foregroundImg;
		vector<ofVec2f> colorCoords;
		int numBodiesTracked;
		bool bHaveAllStreams;

		// helper Functions
		string escape_quotes(const string & before);
		void body2JSON(vector<ofxKinectForWindows2::Data::Body> bodies, const char * jointNames[]);


};
