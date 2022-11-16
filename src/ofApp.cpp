#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(51);
    ofEnableSmoothing();
    ofSetCircleResolution(100);
    //ip = "127.0.0.1";    //getIP();
    ip = getIP();
    port = 8000;
    addres = "/lidar/points";
    sender.setup(ip, port);
    gui.setup("LIDAR Menu");
    gui.add(lidar_range.setup("Max range",3000, 0, 5000));
    gui.add(draw_toggle.setup("distance map", true));
    
    serial.listDevices();
    vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();
    ofSerialDeviceInfo& info = deviceList.at(0);
    cout << info.getDeviceName() << endl;
    auto dev = info.getDevicePath();    ///dev/tty.usbserial-0001
    cout << dev << endl;
    auto sensor = make_shared<ofxRPlidar>();
    sensor->connect(dev);
    isDeviceConnect = sensor->isConnected();
    sensor->start();
    
    //set Particle
    sensors_.push_back(sensor);
}

//--------------------------------------------------------------
void ofApp::update(){
    if(isDeviceConnect){
        for(auto &s : sensors_) {
            s->update();
            auto data = s->getResult();
            ofxOscMessage m;
            m.setAddress(addres);
            points.clear();
            for(auto &d : data) {
                if(d.quality > 0 && d.distance < lidar_range) { //*** distance = cm   ***
                    ofVec2f pos = ofVec2f(d.distance, 0).getRotated(d.angle);
                    m.addIntArg(pos.x);
                    m.addIntArg(pos.y);
                    float x = ofMap(pos.x/2, 0, 5000, 0, ofGetWidth());
                    float y = ofMap(pos.y/2, 0, 5000, 0, ofGetHeight());
                    points.push_back(ofVec2f(x, y));
                }
            }
            sender.sendMessage(m);
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    if(isDeviceConnect){
        if(points.size() > 0){
            ofPushMatrix();
            ofTranslate(ofVec2f(ofGetWidth(), ofGetHeight())/2.f);
            ofRotateDeg(180);
            ofSetColor(200, 0, 0);
            ofDrawCircle(0, 0, 2);
            for(auto p : points){
                ofSetColor(255);
                ofDrawCircle(p.x, p.y, 2);
            }
            ofPopMatrix();
        }
        gui.draw();
        if(draw_toggle) drawLidarArea();
        ofDrawBitmapString("IP: " + ip  + "  PORT: " + ofToString(port) + "  ADDRES: /lidar/points", 5, ofGetHeight()-10);
    } else {
        ofDrawBitmapString("LIDAR disconected. Connect LIDAR and restart this app.", ofGetWidth()/3.5, ofGetHeight()/2);
    }
}

//--------------------------------------------------------------
string ofApp::getIP(){
    int fd;
    struct ifreq ifr;
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name, "en0", IFNAMSIZ-1);
    ioctl(fd, SIOCGIFADDR, &ifr);
    close(fd);
    string ipAddress = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
    //[*] check good ipAddress
    int pos = ipAddress.find_last_of(".");
    string last = ipAddress.substr(pos+1);
    if ((pos==string::npos )|| (last == "0")) ipAddress = "";
    return ipAddress;
}

//--------------------------------------------------------------
void ofApp::drawLidarArea(){
    ofPushStyle();
    ofNoFill();
    ofSetColor(255, 70);
    
    for(int i=150; i<=800; i+=150){
        ofDrawEllipse(ofGetWidth()/2, ofGetHeight()/2, i, i);
        ofDrawBitmapString(ofToString(int(i/150))+'M', 6+ofGetWidth()/2+ cos(ofWrapRadians(45))*i/2,
                        6+ofGetHeight()/2+ sin(ofWrapRadians(45))*i/2);
    }
    ofPopStyle();
}

//--------------------------------------------------------------
void ofApp::exit(){
    serial.close();
}
