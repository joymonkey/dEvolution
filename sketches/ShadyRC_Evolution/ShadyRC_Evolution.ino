/*
 * SHADY RC EVOLUTION : A quick 'n dirty RC setup for droids previously using SHADOW
 * 2019-10-23
 * 
 * Tested with a Turnigy Evolution transmitter and Turnigy TGY-iA6C receiver connected to Serial1 Rx
 * (pin #19 of the Arduino Mega ADK)
 * 
 * ENSURE THAT FAILSAFE MODE WORKS.
 * On your transmitter, go into Settings->Failsafe and enable Failsafe for Channels 1-4
 * Channels should be set to go to 0% when failsafe is detected (ie, when the receiver loses connection
 * with the transmitter)
 * 
 * Channels are set up as follows, mostly using the factory default channel assignmens of the Turnigy Evolution...
 * Right Stick Ch.1 = Steering
 * Right Stick Ch.2 = Throttle
 *  Left Stick Ch.3 = Autodome frequency, or Volume (when knob VrA is fully clockwise)
 *  Left Stick Ch.4 = Dome Rotation
 *  SwB (left) Ch.5 = Set Max Drive Speed   *** Factory default for Ch.5 is both SwB & SwC)
 *   SwA (mid) Ch.6 = Foot Drive Enable (up=off, down=on)
 *  VrA (knob) Ch.7 = sound/function selector
 * SwC (right) Ch.8 = Trigger Sounds   *** Factory default for Ch.8 is disabled!)
 * 
 * Arduino Mega 2560/ADK Pinout
 *   Left Motor PWM : 44
 *  Right Motor PWM : 45
 *   MP3 Trigger Rx : 18 (Serial1 Tx)
 * iBus RC Receiver : 15 (Serial3 Rx)
 * SyRen/Sabertooth : 16 (Serial2 Tx)
 * 
 * MP3 Trigger Note:
 * Files are now referenced by filename instead of file order. MP3 files need to begin with 3 digits, like...
 * 001_doodoo.mp3
 * 002 poopy pants.mp3
 * 084_march.mp3
 * 244 heres a long filename.mp3
 * Only those first 3 digits are what the MP3 Trigger cares about. MP3Trigger library is no longer needed.
 * If your MP3Trigger is very old (pre 2013), you may need a firmware update for this to work.
 * For some reason my MP3Trigger's SD card has an ini file that tells it to run at 9600 baud. I believe the
 * default is 38400 baud, so you may need to change baud rate in this sketch or your ini file to get beepboops.
 * 
 * Libraries required :
 * https://github.com/bmellink/IBusBM
 * https://www.dimensionengineering.com/software/SabertoothArduinoLibraries.zip
 * 
 * Displaying Your Main Battery Level on the Turnigy Evolution:
 * On the TGY-iA6C receiver there is a little 2-pin JST port labeled BDet (short for bidet?). You can connect
 * your main battery here and its voltage will be sent to the transmitter for display. However, the BDet is
 * limited to 4S battery (16.8V) input.
 * If you're using a 6S battery (25.2V), it would damage the receiver, so I connect my main battery using a
 * simple voltage divider using two 10K resistors. Then I tell the Evolution Transmitter to pretend that
 * we've got a 3S battery instead.
 * 
 */

#define FOOT_CONTROLLER 1 //0 for Sabertooth Serial or 1 for individual R/C output (for Q85 motors with 1 controller for each foot, or Sabertooth Mode 2 Independant Mixing)
                          //as of 2019-10-12, no code is in place for Sabertooth Serial

#define SYREN_ADDR         129      // Serial Address for Dome Syren
#define SABERTOOTH_ADDR    128      // Serial Address for Foot Sabertooth (if used)

#define DEBUG 1           //1 to enable debug mode (may make control choppy), 0 to disable 

#define Serial_MP3 Serial1      //Serial port used by the Sparkfun MP3 Trigger 
#define Serial_Receiver Serial3 //Serial port used by the iBus RC receiver
#define Serial_ST Serial2       //Serial port used by Sabertooth (feet) and/or Syren (dome)
#define Serial_Debug Serial     //Serial port used by for debug messages
#define invertDome 1      //change this to a 1 or 0 to change the direction that the dome spins  
#define leftFootPin 44    //connect this pin to motor controller for left foot (R/C mode)
#define rightFootPin 45   //connect this pin to motor controller for right foot (R/C mode)
#define invertLeft  0     //foot spinning the wrong way? change this to 1 or 0
#define invertRight 0     //foot spinning the wrong way? change this to 1 or 0
#define joystickFootDeadZoneRange 10
int domeRotationSpeed,prevDomeRotationSpeed;
#define motorControllerBaudRate 9600 //bjaud rate for Syren and Sabertooth

byte vol = 50; // initial volume level, 0 = full volume, 255 off
#define minVolume 100   //255=silent, anything over 100 is too quiet for my liking so I never go there
#define maxVolume 0     //0=loudest, most obnoxious droid in the room
byte maxDriveSpeed=30;  //Sabertooth uses a value of 0 to 127 for drive speed
byte driveSpeeds[3] = {30,80,127};  //these are the drivespeeds that we can toggle between (0=statue, 127=ludicrous)
#define turnspeed 50    //turnspeed is used by the mixing algorythm. might look into making this more dynamic in the future.
int leftFoot,rightFoot; //will hold foot speed values (-100 to 100)
boolean isStickEnabled = false; //changed to false so the feet don't take off immediately on power up (oops)
unsigned long previousDomeMillis = millis();
unsigned long previousFootMillis = millis();
unsigned long currentMillis = millis();
#define serialFootLatency 25  //This is a delay factor in ms to prevent queueing of the Serial data.
#define serialDomeLatency 25  //This is a delay factor in ms to prevent queueing of the Serial data.
                          //25ms seems appropriate for HardwareSerial, values of 50ms or larger are needed for Softare Emulation
int txChannels[9];        //This holds our transmitter channel values. we have 8 channels (1-8) but this array has 9 values (0-8),
                          //so to prevent confussion we ignore the first (0) value.
int prevTxChannels[9];
unsigned long prevManualDomeMove;
bool isAutodomeEnabled=false;
unsigned long previousAutodomeMillis = millis();
unsigned long autodomeInterval;
int currentAutodomeSpeed;
byte currentAutodomeState; //0=waiting, 1=initial move, 2=return move
#define maxAutodomeSpeed 50
#define minAutodomeSpeed 15
unsigned int autodomeMoveMillis;
#define maxAutodomeMoveMillis 1000 //how long the motor spends spinning on each autodome move
#define minAutodomeMoveMillis 500
#define autoChatty 6 //how chatty do you want your droid? 10 will beepboop on almost every autodome move, 1 should be fairly rarely

#include <Sabertooth.h>
#if FOOT_CONTROLLER != 1 
  Sabertooth *ST=new Sabertooth(SABERTOOTH_ADDR, Serial_ST); //we'll use a Sabertooth motor controller
  // ** CODE FOR SABERTOOTH FOOT DRIVE IS NOT IN PLACE, FEEL FREE TO WORK ON IT! ** //
#endif
Sabertooth *SyR=new Sabertooth(SYREN_ADDR, Serial_ST);

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1052__) || defined(__IMXRT1062__)
  #include <PWMServo.h> //regular Servo library doesn't work on Teensy, so we use PWMServo instead
  PWMServo leftFootSignal;
  PWMServo rightFootSignal; 
#else
  #include <Servo.h>
  Servo leftFootSignal;
  Servo rightFootSignal; 
#endif

//library used for iBus RC receiver...
#include <IBusBM.h>
IBusBM IBus; 

// quick function to stop the feet depending on which drive system we're using...
void stopFeet() {
  #if FOOT_CONTROLLER == 0
  ST->stop();
  #elif FOOT_CONTROLLER == 1
  leftFootSignal.write(90);
  rightFootSignal.write(90);
  #endif
}

void footMotorDrive() {
  //Flood control prevention
  if ((currentMillis-previousFootMillis)<serialFootLatency) return;  
  else if (isStickEnabled==1) {
    //it's been a moment since we thought about doing anything with the feet. Do that thing.
    previousFootMillis=currentMillis;
    //joystick values will be between 1000 and 2000 (1500 is centered)
    #if FOOT_CONTROLLER == 1
      //map the joystick values down so the mixBHD function will like them
      mixBHD(map(txChannels[1],1000,2000,0,255), map(txChannels[2],2000,1000,0,255), map(maxDriveSpeed,0,127,90,180)); //put values into leftFoot and rightFoot
        #if (DEBUG==1)
          Serial_Debug.print(leftFoot);
          Serial_Debug.print(F(","));
          Serial_Debug.println(rightFoot);
        #endif
        //now move those feet!       
        leftFootSignal.write(leftFoot);
        rightFootSignal.write(rightFoot);
    #endif
    // ** TO-DO! PUT STUFF HERE FOR PEOPLE WITH SABERTOOTH SERIAL FOOT DRIVE
  }
} 

void playMP3(byte MP3num) {
  //previously SHADOW used the trigger.play(53) command to play MP3's
  //see https://learn.sparkfun.com/tutorials/mp3-trigger-hookup-guide-v24/all#serial-control-protocol- for full details
  Serial_MP3.write('t'); //should this be 't' (binary) or 'T' (ASCII) ?
  Serial_MP3.write(MP3num);
  #if (DEBUG==1)
    Serial_Debug.write('t');
    Serial_Debug.println(MP3num);
  #endif
}

void volume(byte level) {
  Serial_MP3.write('v');
  Serial_MP3.write(level);
  #if (DEBUG==1)
    Serial_Debug.print('v');
    Serial_Debug.println(level);
  #endif
}

void domeDrive() {
  //Flood control prevention
  if ((currentMillis-previousDomeMillis) < serialDomeLatency || (txChannels[4]<1000 || txChannels[4]>2000)) return; 
  if (txChannels[4]!=prevTxChannels[4]) {
    //cancel autodome if the stick has moved
    prevManualDomeMove=currentMillis;
    isAutodomeEnabled=false; 
    currentAutodomeSpeed=0;
    currentAutodomeState=1; //0=waiting, 1=initial move, 2=return move
  }
  if (isAutodomeEnabled==false) {
    #if (invertDome==0)
      domeRotationSpeed=map(txChannels[4],1000,2000,-127,127);
    #else
      domeRotationSpeed=map(txChannels[4],1000,2000,127,-127);
    #endif
  }  
  else {
    //Autodome is on, bitches! Determine if we're rotating right now or waiting
    if (currentMillis-previousAutodomeMillis>autodomeInterval) {
      #if (DEBUG==1)
        Serial_Debug.println("autodome!");
      #endif
      //if (currentAutodomeSpeed==0) {
      if (currentAutodomeState==1) {
        autodomeMoveMillis=random(minAutodomeMoveMillis,maxAutodomeMoveMillis); //a random length of time to spin
        currentAutodomeSpeed=random(minAutodomeSpeed,maxAutodomeSpeed); //a random dome speed
        if (random(2)==0) currentAutodomeSpeed=currentAutodomeSpeed*-1;  //randomly decide to go the other way perhaps maybe
        #if (DEBUG==1)
          Serial_Debug.print("initial spd ");
          Serial_Debug.println(currentAutodomeSpeed);
        #endif
        currentAutodomeState=0;
      }
      //else if (currentAutodomeSpeed>0) {
      else if (currentAutodomeState==0) {
        //last time we autodomed the dome was moved forward, lets move it back
        currentAutodomeSpeed=currentAutodomeSpeed*-1;
        #if (DEBUG==1)
          Serial_Debug.print("return spd ");
          Serial_Debug.println(currentAutodomeSpeed);
        #endif
        currentAutodomeState=1;
      }
      else { 
        currentAutodomeState=2;
        #if (DEBUG==1)
          Serial_Debug.println("hold up");
        #endif
      }
      /*else {
        currentAutodomeSpeed=0; //does this ever really happen?
        #if (DEBUG==1)
          Serial_Debug.println("hold up");
        #endif
      }*/
      previousAutodomeMillis=currentMillis;
      if (random(0,21)<=autoChatty) playMP3(random(0,19)); //play a random general beepboop maybe
    }
    if (currentMillis-previousAutodomeMillis<autodomeMoveMillis) {
      ////motor is moving!
      domeRotationSpeed=currentAutodomeSpeed;
    }
    else domeRotationSpeed=0;
  }
  if (domeRotationSpeed!=prevDomeRotationSpeed) {
    SyR->motor(domeRotationSpeed);  
    #if (DEBUG==1)
      if (domeRotationSpeed!=0) {
        //Serial_Debug.print(txChannels[4]);
        Serial_Debug.print(F("d"));
        Serial_Debug.println(domeRotationSpeed);
      }
    #endif
  }
  prevDomeRotationSpeed=domeRotationSpeed;
}  

#if FOOT_CONTROLLER == 1
void mixBHD(byte stickX, byte stickY, byte maxDriveSpeed){  //maxDriveSpeed should be between 90 and 180
    // This is BigHappyDude's mixing function, for differential (tank) style drive using two motor controllers.
    // Takes a joysticks X and Y values, mixes using the diamond mix, and output a value 0-180 for left and right motors.     
    // 180,180 = both feet full speed forward.
    // 000,000 = both feet full speed reverse.
    // 180,000 = left foot full forward, right foot full reverse (spin droid clockwise)
    // 000,180 = left foot full reverse, right foot full forward (spin droid counter-clockwise)
    // 090,090 = no movement
    // for simplicity, we think of this diamond matrix as a range from -100 to +100 , then map the final values to servo range (0-180) at the end 
    //  Ramping and Speed mode applied on the droid.  
    if(((stickX <= 113) || (stickX >= 141)) || ((stickY <= 113) || (stickY >= 141))){  //  if movement outside deadzone
      //  Map to easy grid -100 to 100 in both axis, including deadzones.
      int YDist = 0;  // set to 0 as a default value if no if used.
      int XDist = 0;
      if(stickY <= 113){
       YDist = (map(stickY, 0, 113, 100, 1));           //  Map the up direction stick value to Drive speed
      } else if(stickY >= 141){
       YDist = (map(stickY, 141, 255, -1, -100));       //  Map the down direction stick value to Drive speed
      }
      if(stickX <= 113){
       XDist = (map(stickX, 0, 113, -turnspeed, -1));       //  Map the left direction stick value to Turn speed
      } else if(stickX >= 141){
       XDist = (map(stickX, 141, 255, 1, turnspeed));   //  Map the right direction stick value to Turn speed
      }
      //  Constrain to Diamond values.  using 2 line equations and find the intersect, boiled down to the minimum
      //  This was the inspiration; https://github.com/declanshanaghy/JabberBot/raw/master/Docs/Using%20Diamond%20Coordinates%20to%20Power%20a%20Differential%20Drive.pdf 
      float TempYDist = YDist;
      float TempXDist = XDist;
      if (YDist>(XDist+100)) {  //  if outside top left.  equation of line is y=x+Max, so if y > x+Max then it is above line
        // OK, the first fun bit. :)  so for the 2 lines this is always true y = m1*x + b1 and y = m2*x - b2
        // y - y = m1*x + b1  - m2*x - b2  or 0 = (m1 - m2)*x + b1 - b2
        // We have y = x+100 and y = ((change in y)/Change in x))x
        // So:   x = -100/(1-(change in y)/Change in x)) and using y = x+100 we can find y with the new x
        // Not too bad when simplified. :P
        TempXDist = -100/(1-(TempYDist/TempXDist));
        TempYDist = TempXDist+100;
      } else if (YDist>(100-XDist)) {  //  if outside top right
        // repeat intesection for y = 100 - x
        TempXDist = -100/(-1-(TempYDist/TempXDist));
        TempYDist = -TempXDist+100;
      } else if (YDist<(-XDist-100)) {  //  if outside bottom left
        // repeat intesection for y = -x - 100
        TempXDist = 100/(-1-(TempYDist/TempXDist));
        TempYDist = -TempXDist-100;
      } else if (YDist<(XDist-100)) {  //  if outside bottom right
        // repeat intesection for y = x - 100
        TempXDist = 100/(1-(TempYDist/TempXDist));
        TempYDist = TempXDist-100;
      }
      //  all coordinates now in diamond. next translate to the diamond coordinates.
      //  for the left.  send ray to y = x + Max from coordinates along y = -x + b
      //  find for b, solve for coordinates and resut in y then scale using y = (y - max/2)*2
      float LeftSpeed = ((TempXDist+TempYDist-100)/2)+100;
      LeftSpeed = (LeftSpeed-50)*2;
      //  for right send ray to y = -x + Max from coordinates along y = x + b find intersction coordinates and then use the Y vaule and scale.
      float RightSpeed = ((TempYDist-TempXDist-100)/2)+100;
      RightSpeed = (RightSpeed-50)*2;
      // this all results in a -100 to 100 range of speeds, so shift to servo range...
      //  eg. for a maxDriveSpeed of 140, we'd need the value to map to between 40 and 140
      //  eg. for a maxDriveSpeed of 180, we'd need the value to map to between 0 and 180
      //leftFoot=map(LeftSpeed, -100, 100, (180-maxDriveSpeed), maxDriveSpeed);
      //rightFoot=map(RightSpeed, -100, 100, (180-maxDriveSpeed), maxDriveSpeed);
      #if invertLeft == 0
        leftFoot=map(LeftSpeed, 100, -100, maxDriveSpeed, (180-maxDriveSpeed) );
      #else
        leftFoot=map(LeftSpeed, -100, 100, maxDriveSpeed, (180-maxDriveSpeed) );
      #endif
      #if invertRight == 0
        rightFoot=map(RightSpeed, 100, -100, maxDriveSpeed, (180-maxDriveSpeed) );
      #else
        rightFoot=map(RightSpeed, -100, 100, maxDriveSpeed, (180-maxDriveSpeed) );
      #endif
    } else {
      leftFoot=90;
      rightFoot=90;
    }
}
#endif

void getTransmitterValues() {
  for (int i=1; i<9 ; i++) {
    prevTxChannels[i]=txChannels[i];
    txChannels[i]=IBus.readChannel(i-1);
    if (i<5 && (txChannels[i]<1500+joystickFootDeadZoneRange && txChannels[i]>1500-joystickFootDeadZoneRange)) {
      //stick is within the deadband, treat it as centered
      txChannels[i]=1500;
    }
    #if (DEBUG==1)
      if (i>4 && txChannels[i]!=prevTxChannels[i]) {
        Serial_Debug.print("ch");
        Serial_Debug.print(i);
        Serial_Debug.print("=");
        Serial_Debug.println(txChannels[i]);
      }
    #endif
  }
}

void setup() {  

  randomSeed(analogRead(A4));

  #if (DEBUG==1)
    Serial_Debug.begin(115200);  // initialize serial port for debug
    Serial_Debug.println(F("setup..."));
  #endif

  IBus.begin(Serial_Receiver); // initialize serial port for the receiver
  #if (DEBUG==1)
    Serial_Debug.println(F("/iBus"));
  #endif

  /*#if FOOT_CONTROLLER == 1
    leftFootSignal.attach(leftFootPin);
    rightFootSignal.attach(rightFootPin);
    // 2019-10-09 : removed this; signals now attach/detach when drive stick is enabled/disabled
  #endif*/
  stopFeet();
  #if (DEBUG==1)
    Serial_Debug.println(F("/servos"));
  #endif

  Serial_ST.begin(motorControllerBaudRate);

  Serial_MP3.begin(9600); //default baud was 38400, my trigger has an INI file that sets it to 9600 for some reason
  delay(1250); //the mp3trigger takes a second to start up, so we wait before setting its volume
  volume(vol);
  delay(50);
  playMP3(255);

  #if (DEBUG==1)
    Serial_Debug.println(F("/setup"));
  #endif

}

void loop() {

  currentMillis=millis();

  getTransmitterValues();

  if (txChannels[6]!=prevTxChannels[6]) {
    #if (DEBUG==1)
      Serial_Debug.print("drv ");
    #endif
    if (txChannels[6]==2000) {
      isStickEnabled = true; 
      leftFootSignal.attach(leftFootPin);
      rightFootSignal.attach(rightFootPin);
      #if (DEBUG==1)
        Serial_Debug.println("on");
      #endif
    }
    else {
      isStickEnabled = false; 
      leftFootSignal.detach();
      rightFootSignal.detach();
      #if (DEBUG==1)
        Serial_Debug.println("off");
      #endif
    }
  }

  if (txChannels[5]!=prevTxChannels[5]) {
    if (txChannels[5]==1000) maxDriveSpeed=driveSpeeds[0];      //slowest
    else if (txChannels[5]==1500) maxDriveSpeed=driveSpeeds[1]; //mid
    else maxDriveSpeed=driveSpeeds[2];    //fastest
    #if (DEBUG==1)
      Serial_Debug.print("maxSpd");
      Serial_Debug.println(maxDriveSpeed);
    #endif
  }

  if (txChannels[8]!=prevTxChannels[8]) {
    if (txChannels[8]==1500) { /*switch is centered, do nuthin*/ }
    else if (txChannels[8]==1000) {
      //play a random beepboop
      playMP3(random(0,19));
    }
    else if (txChannels[8]==2000) {
      if (txChannels[7]==1000) {
        playMP3(random(76,80)); //if we're at the very end play a random sad sound
      }
      else if (txChannels[7]==2000) {
        playMP3(31); //laugh
      }
      else {
        //play a specific sound based on the knob position (knob value from 1980 to 1020)
        //aside from the extremities, we've got 13 knob positions. each one occupies 60 values of the knob 
        #define numChoons 13
        #define upperKnobVal 2000
        #define lowerKnobVal 1000
        #define knobStep (upperKnobVal-lowerKnobVal)/numChoons
        byte choons[numChoons] = {152,183,128,201,202,203,204,205,206,207,208,209,31};
        int knobval=upperKnobVal;      
        byte choonNum=0;
        while (knobval > (upperKnobVal-(numChoons*knobStep)) ) {
          if (txChannels[7]<=knobval&&txChannels[7]>(knobval-knobStep)) {
            playMP3(choons[choonNum]); 
          }
          knobval=knobval-knobStep;
          choonNum++;
        }
      }
    }
  }

  if (txChannels[3]!=prevTxChannels[3] && txChannels[7]==2000) {
    //use the left stick's vertical axis (ch3) to set volume
    //all the way up ch3=2000
    volume( map(txChannels[3],1000,2000,minVolume,maxVolume) ); //volume levels between 100 and 255 are too quiet
  }

  if (txChannels[1]!=prevTxChannels[1] || txChannels[2]!=prevTxChannels[2]) {
    #if (DEBUG==1)
      Serial_Debug.print(txChannels[1]);
      Serial_Debug.print(",");
      Serial_Debug.println(txChannels[2]);
    #endif
    footMotorDrive();
  }

  if (txChannels[3]>1020 && txChannels[3]<1950 && txChannels[7]<1950 && txChannels[3]!=prevTxChannels[3] && currentMillis-prevManualDomeMove>5000) {
    //use the left stick's vertical axis (ch3) to set autodomeInterval
    //all the way up ch3=2000
    isAutodomeEnabled=true;
    //autodomeInterval=txChannels[3]*5; //sets frequency of autodome moves based on the left sticks vertical position
    autodomeInterval=map(txChannels[3],1000,2000,2000*5,1000); //sets frequency of autodome moves based on the left sticks vertical position  
    #if (DEBUG==1)
      //Serial_Debug.print("interval");
      //Serial_Debug.println(autodomeInterval);
    #endif
  }
  if ((txChannels[3]!=prevTxChannels[3] && txChannels[8]!=1500) || txChannels[3]<1005 || txChannels[7]==2000)  {
    isAutodomeEnabled=false;
    #if (DEBUG==1)
      Serial_Debug.println("autodome disabled");
    #endif
  }
  domeDrive();
  
}
