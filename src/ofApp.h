#pragma once

#include "ofMain.h"
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>

#include "ofxGui.h"
#include "ofxOsc.h"
#include "ofxRPlidar.h"


class ofApp : public ofBaseApp{
public:
    void setup();
    void update();
    void draw();
    void drawLidarArea();
    void exit();

//    void keyPressed(int key);
//    void keyReleased(int key);
//    void mouseDragged(int x, int y, int button);
//    void mousePressed(int x, int y, int button);
//    void mouseReleased(int x, int y, int button);
    string getIP();
    
    
    ofxFloatSlider lidar_range;
    ofxToggle draw_toggle;
    ofxPanel gui;
    ofxOscSender sender;
    string ip, addres;
    int port;
    bool isDeviceConnect;
    std::vector<ofVec2f> points;
private:
    std::vector<std::shared_ptr<ofxRPlidar>> sensors_;
    ofSerial serial;
};
