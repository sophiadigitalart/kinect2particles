#include "ofApp.h"

/*
*  kinect2share
*
*  Created by Ryan Webber
*  http://www.DusXproductions.com
*  https://github.com/rwebber
*
*  The goal of this project is to make as many Kinect2 features available to creative platforms as possible,
*  using open standards including OSC (opensoundcontrol), Spout, and NDI (https://www.newtek.com/ndi/).
*
*  Specific care has been given to providing a demo file for use with the Isadora creativity server.
*  The demo file provides basic functional examples that Isadora users can build upon.
*  http://troikatronix.com/
*
*  MIT License http://en.wikipedia.org/wiki/MIT_License
*
*  This project is built using OpenFrameWorks and utilizes a number of amazing addons offered by the community.
*  Please read the ReadMe file included in the github reprository, for details.
*/

#define DEPTH_WIDTH 512
#define DEPTH_HEIGHT 424
#define DEPTH_SIZE DEPTH_WIDTH * DEPTH_HEIGHT

#define COLOR_WIDTH 1920
#define COLOR_HEIGHT 1080

int previewWidth = DEPTH_WIDTH / 2; // width and hieght of Depth Camera scaled
int previewHeight = DEPTH_HEIGHT / 2;

string guiFile = "settings.xml";

// REF: http://www.cplusplus.com/reference/cstring/

// TODO: look into https://forum.openframeworks.cc/t/ofxkinectforwindows2-depth-threshold-for-blob-tracking/19012/2

// TODO: refactor 'kv2status' into user defineable address

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetWindowTitle("kinect2osc");
	ofSetFrameRate(20);
	ofSetVerticalSync(true);

	color_StreamName = "kv2_color";
	cutout_StreamName = "kv2_cutout";
	depth_StreamName = "kv2_depth";
	keyed_StreamName = "kv2_keyed";

	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();

	// added for coordmapping
	if (kinect.getSensor()->get_CoordinateMapper(&coordinateMapper) < 0) {
		ofLogError() << "Could not acquire CoordinateMapper!";
	}
	numBodiesTracked = 0;
	bHaveAllStreams = false;
	foregroundImg.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR_ALPHA);
	colorCoords.resize(DEPTH_WIDTH * DEPTH_HEIGHT);
	// end add for coordmapping

	ofSetWindowShape(previewWidth * 3, previewHeight * 2);

	//ofDisableArbTex(); // needed for textures to work... May be needed for NDI?
	// seems above is needed if loading an image file -> texture
	bgCB.load("images/checkerbg.png");

	// TODO: depth and IR to be added -> fboDepth
	fboDepth.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, GL_RGBA); //setup offscreen buffer in openGL RGBA mode (used for Keyed and B+W bodies.)
	fboColor.allocate(COLOR_WIDTH, COLOR_HEIGHT, GL_RGB); //setup offscreen buffer in openGL RGB mode


	// GUI SETUP ***************** http://openframeworks.cc/documentation/ofxGui/
	gui.setup("Parameters", guiFile);
	gui.setHeaderBackgroundColor(ofColor::darkRed);
	gui.setBorderColor(ofColor::darkRed);
	//gui.setDefaultWidth(320);

	OSCgroup.setup("OSC");
	OSCgroup.add(jsonGrouped.setup("OSC as JSON", true));
	OSCgroup.add(HostField.setup("Host ip", "localhost"));
	OSCgroup.add(oscPort.setup("Output port", 7000));
	OSCgroup.add(oscPortIn.setup("Input port", 4321));
	gui.add(&OSCgroup);


	gui.loadFromFile(guiFile);

	// OSC setup  * * * * * * * * * * * * *
	//oscSender.disableBroadcast();
	oscSender.setup(HostField, oscPort);
	oscReceiver.setup(oscPortIn);

}

//--------------------------------------------------------------
void ofApp::update() {
	// get OSC messages
	while (oscReceiver.hasWaitingMessages()) {
		ofxOscMessage m;
		oscReceiver.getNextMessage(&m);

		if (m.getAddress() == "/app-exit") {
			bool trigger = m.getArgAsInt(0);
			//string s = m.getArgAsString(0);
			if (trigger) {
				exit();
				std::exit(0);
				oscSendMsg("exit", "/kv2status/");
			}
		}
	}

	//KV2
	kinect.update();

	// Get pixel data
	auto& depthPix = kinect.getDepthSource()->getPixels();
	auto& bodyIndexPix = kinect.getBodyIndexSource()->getPixels();
	auto& colorPix = kinect.getColorSource()->getPixels();

	// Make sure there's some data here, otherwise the cam probably isn't ready yet
	if (!depthPix.size() || !bodyIndexPix.size() || !colorPix.size()) {
		bHaveAllStreams = false;
		return;
	}
	else {
		bHaveAllStreams = true;
	}

	// Count number of tracked bodies
	numBodiesTracked = 0;
	auto& bodies = kinect.getBodySource()->getBodies();
	for (auto& body : bodies) {
		if (body.tracked) {
			numBodiesTracked++;
		}
	}

	// Do the depth space -> color space mapping
	// More info here:
	// https://msdn.microsoft.com/en-us/library/windowspreview.kinect.coordinatemapper.mapdepthframetocolorspace.aspx
	// https://msdn.microsoft.com/en-us/library/dn785530.aspx
	coordinateMapper->MapDepthFrameToColorSpace(DEPTH_SIZE, (UINT16*)depthPix.getPixels(), DEPTH_SIZE, (ColorSpacePoint*)colorCoords.data());


	// Update the images since we manipulated the pixels manually. This uploads to the
	// pixel data to the texture on the GPU so it can get drawn to screen
	foregroundImg.update();

	//--
	//Getting joint positions (skeleton tracking)
	//--
	//

	 /******************  ENUM copied from kinectv2 addon
	  JointType_SpineBase = 0,
		JointType_SpineMid = 1,
		JointType_Neck = 2,
		JointType_Head = 3,
		JointType_ShoulderLeft = 4,
		JointType_ElbowLeft = 5,
		JointType_WristLeft = 6,
		JointType_HandLeft = 7,
		JointType_ShoulderRight = 8,
		JointType_ElbowRight = 9,
		JointType_WristRight = 10,
		JointType_HandRight = 11,
		JointType_HipLeft = 12,
		JointType_KneeLeft = 13,
		JointType_AnkleLeft = 14,
		JointType_FootLeft = 15,
		JointType_HipRight = 16,
		JointType_KneeRight = 17,
		JointType_AnkleRight = 18,
		JointType_FootRight = 19,
		JointType_SpineShoulder = 20,
		JointType_HandTipLeft = 21,
		JointType_ThumbLeft = 22,
		JointType_HandTipRight = 23,
		JointType_ThumbRight = 24,
		JointType_Count = (JointType_ThumbRight + 1)
		*/


		// shorten names to minimize packet size
	const char * jointNames[] = { "SpineBase", "SpineMid", "Neck", "Head",
		"ShldrL", "ElbowL", "WristL", "HandL",
		"ShldrR", "ElbowR", "WristR", "HandR",
		"HipL", "KneeL", "AnkleL", "FootL",
		"HipR", "KneeR", "AnkleR", "FootR",
		"SpineShldr", "HandTipL", "ThumbL", "HandTipR", "ThumbR", "Count" };

	/* MORE joint. values >>>
	 second. positionInWorld[] x y z , positionInDepthMap[] x y
	 second. orientation. _v[] x y z w  ??what is this
	 second. trackingState
	 */

	 /* MORE body. values >>>
	 body. tracked (bool)
	 body. leftHandState (_Handstate) enum?
	 body. rightHandState (_Handstate)
	 body. activity  ??what is this
	 */

	 // defined in new coordmap section as &
	 //	auto bodies = kinect.getBodySource()->getBodies();


	
		// TODO:: seperate function and add additional features like hand open/closed
		// NON JSON osc messages
		for (auto body : bodies) {
			for (auto joint : body.joints) {
				auto pos = joint.second.getPositionInWorld();
				ofxOscMessage m;
				string adrs = "/" + to_string(body.bodyId) + "/" + jointNames[joint.first];
				m.setAddress(adrs);
				m.addFloatArg(pos.x);
				m.addFloatArg(pos.y);
				m.addFloatArg(pos.z);
				m.addIntArg(joint.first);
				m.addIntArg(body.bodyId);
				m.addStringArg(jointNames[joint.first]);
				oscSender.sendMessage(m);
				//cout << adrs << endl;
				ofLogError() << adrs;
				ofLogNotice() << "osc send " << adrs;
			} // end inner joints loop
		} // end body loop

	

	//--
	//Getting bones (connected joints)
	//--
	//

	{
		//// Note that for this we need a reference of which joints are connected to each other.
		//// We call this the 'boneAtlas', and you can ask for a reference to this atlas whenever you like
		//auto bodies = kinect.getBodySource()->getBodies();
		//auto boneAtlas = ofxKinectForWindows2::Data::Body::getBonesAtlas();

		//for (auto body : bodies) {
		//	for (auto bone : boneAtlas) {
		//		auto firstJointInBone = body.joints[bone.first];
		//		auto secondJointInBone = body.joints[bone.second];

		//		//now do something with the joints
		//	}
		//}
	}

	//
	//--
}


//--------------------------------------------------------------
void ofApp::draw() {
	stringstream ss;

	ofClear(0, 0, 0);
	bgCB.draw(0, 0, ofGetWidth(), ofGetHeight());

	// Color is at 1920x1080 instead of 512x424 so we should fix aspect ratio
	float colorHeight = previewWidth * (kinect.getColorSource()->getHeight() / kinect.getColorSource()->getWidth());
	float colorTop = (previewHeight - colorHeight) / 2.0;

	{
		// Draw Depth Source
		// TODO: brighten depth image. https://github.com/rickbarraza/KinectV2_Lessons/tree/master/3_MakeRawDepthBrigther
		// MORE: https://forum.openframeworks.cc/t/kinect-v2-pixel-depth-and-color/18974/4 
		fboDepth.begin(); // start drawing to off screenbuffer
		ofClear(255, 255, 255, 0);
		kinect.getDepthSource()->draw(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT);  // note that the depth texture is RAW so may appear dark
		fboDepth.end();
		
		//Draw from FBO
		fboDepth.draw(0, 0, previewWidth, previewHeight);
		//fboDepth.clear();
	}

	{
		// Draw Color Source
		fboColor.begin(); // start drawing to off screenbuffer
		ofClear(255, 255, 255, 0);
		kinect.getColorSource()->draw(0, 0, COLOR_WIDTH, COLOR_HEIGHT);
		fboColor.end();
		
		//Draw from FBO to UI
		fboColor.draw(previewWidth, 0 + colorTop, previewWidth, colorHeight);
		//fboColor.clear();
	}

	{
		// Draw IR Source
		kinect.getInfraredSource()->draw(0, previewHeight, DEPTH_WIDTH, DEPTH_HEIGHT);
		//kinect.getLongExposureInfraredSource()->draw(0, previewHeight, previewWidth, previewHeight);
	}

	{
		// Draw B+W cutout of Bodies
		fboDepth.begin(); // start drawing to off screenbuffer
		ofClear(255, 255, 255, 0);
		kinect.getBodyIndexSource()->draw(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT);
		fboDepth.end();

		//Draw from FBO
		fboDepth.draw(previewWidth, previewHeight, previewWidth, previewHeight);
		//fboDepth.clear();
	}

	{
		// greenscreen/keyed fx from coordmaping
		fboDepth.begin(); // start drawing to off screenbuffer
		ofClear(255, 255, 255, 0);
		foregroundImg.draw(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT);
		fboDepth.end();
		
			ss.str("");
			ss << "Keyed image only shown when" << endl;
			ss << "checked in Parameters window" << endl;
			ss << "and, a body is being tracked.";
			ofDrawBitmapStringHighlight(ss.str(), previewWidth * 2 + 20, previewHeight - (previewHeight / 2 + 60));
		
	}

	{
		// Draw bodies joints+bones over
		kinect.getBodySource()->drawProjected(previewWidth * 2, previewHeight, previewWidth, previewHeight, ofxKFW2::ProjectionCoordinates::DepthCamera);
	}

	ss.str("");
	ss << "fps : " << ofGetFrameRate();
	if (!bHaveAllStreams) ss << endl << "Not all streams detected!";
	ofDrawBitmapStringHighlight(ss.str(), 20, previewHeight * 2 - 25);

	ss.str("");
	ss << "Keyed FX : cpu heavy";
	ofDrawBitmapStringHighlight(ss.str(), previewWidth * 2 + 20, 20);

	ss.str("");
	ss << "Color : HD 1920x1080";
	ofDrawBitmapStringHighlight(ss.str(), previewWidth + 20, 20);

	ss.str("");
	ss << "BnW : body outlines";
	ofDrawBitmapStringHighlight(ss.str(), previewWidth + 20, previewHeight + 20);

	ss.str("");
	ss << "Bodies : coordinates -> OSC" << endl;
	ss << "Tracked bodies: " << numBodiesTracked;
	ofDrawBitmapStringHighlight(ss.str(), previewWidth * 2 + 20, previewHeight + 20);

	ss.str("");
	ss << "Depthmap : ";
	ofDrawBitmapStringHighlight(ss.str(), 20, 20);

	ss.str("");
	ss << "Infrared : ";
	ofDrawBitmapStringHighlight(ss.str(), 20, previewHeight + 20);

	gui.draw();
}

void ofApp::exit() {
	gui.saveToFile(guiFile);
	oscSendMsg("closed", "/kv2status/");
}

//--- OSC send message -----------------------------------------------------------
// Requires the address be wrapped in forward slashes: eg "/status/"
void ofApp::oscSendMsg(string message, string address)
{
	// send OSC message // oscSendMsg("no device","/status/");
	ofxOscMessage m;
	m.setAddress(address);
	m.addStringArg(message);
	oscSender.sendMessage(m);
}

string ofApp::escape_quotes(const string &before)
// sourced from: http://stackoverflow.com/questions/1162619/fastest-quote-escaping-implementation
{
	string after;
	after.reserve(before.length() + 4);  // TODO: may need to increase reserve...

	for (string::size_type i = 0; i < before.length(); ++i) {
		switch (before[i]) {
		case '"':
		case '\\':
			after += '\\';
			// Fall through.
		default:
			after += before[i];
		}
	}
	return after;
}

//--------------------------------------------------------------
void ofApp::HostFieldChanged() {
	cout << "fieldChange" << endl;
	//oscSender.disableBroadcast();
	oscSender.setup(HostField, oscPort);
	oscReceiver.setup(oscPortIn);
	cout << "updated" << endl;
	oscSendMsg("fieldUpdated", "/kv2status/");
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
	// https://forum.openframeworks.cc/t/keypressed-and-getting-the-special-keys/5727
	if (key == OF_KEY_RETURN) {
		cout << "ENTER" << endl;
		HostFieldChanged();
	}
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	ofxOscMessage m;
	string adrs = "/mousemove/1";
	m.setAddress(adrs);
	m.addIntArg(x);
	m.addIntArg(y);
	oscSender.sendMessage(m);
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {
	// textField = ofToString(w) + "x" + ofToString(h);
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}