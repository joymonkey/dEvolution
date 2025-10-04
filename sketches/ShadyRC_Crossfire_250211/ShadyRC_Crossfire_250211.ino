/* ShadyRC Crossfire
   an update to ShadyRC_dEvolution, using Crossfire/ELRS radios

   Can be used with most EdgeTX transmitters using an ExpressLRS or Crossfire radio
   Heavily tested with a Radiomaster Zorro (running EdgeTX v2.9.0), internal ExpressLRS 2.4Ghz radio running ExpressLRS v3.3.0
   Using a DIY 2.4Ghz receiver on the droid side

We have 16 channels to use...

CHANNEL : FUNCTION : ZORRO SWITCH
     01 : Directn  : Right Stick X
     02 : Throttle : Right Stick Y
     03 : AutoDome : Left Stick Y
     04 : DomeRot. : Left Stick X
     05 : Arm      : SE (2-way)
     06 :          : SF (2-way)
     07 : Speed    : SB (3-way)
     08 :          : SC (3-way)
     09 : PlayTune : SA (momentary)
     10 : PlayBeep : SD (momentary)
     11 : 
     12 : SoundSel : S1 (dial)
     13 : Volume   : S2 (dial)
     14 : PlayTune2: SG
     15 : 
     16 : 

*/

//                                                            
#include <AlfredoCRSF.h>     // Library forked from CRServoF example https://github.com/AlfredoSystems/AlfredoCRSF     
#include <Sabertooth.h>      // NOT the stock Dimension Engineering library! Get it from https://github.com/dominicklee/Sabertooth-for-ESP32
#include <ESP32Servo.h>      // regular Arduino Servo library won't work, so we go with this one by John K. Bennett
#include <HardwareSerial.h>  
#include <SoftwareSerial.h>  // EspSoftwareSerial library by Dirk Kaar, should be installable from the IDE https://github.com/plerup/espsoftwareserial/
//ESP32 Serial rx/tx can be assigned any pins, but will have direct access with these pins...
//  ESP32 UART0 (Serial)  RX=GPIO3  TX=GPIO1  This uart is used to print debug info over USB
//  ESP32 UART1 (Serial1) RX=GPIO26 TX=GPIO27 This uart is reserved for the Syren and Rome-A-Dome
//  ESP32 UART2 (Serial2) RX=GPIO16 TX=GPIO17 This uart is used to talk to the ELRS receiver (using Crossfire protocol)
// we use software serial for additional serial ports; to talk to the MP3 Trigger, maybe to get info from Roboteq's, send commands to the dome etc
//#define Serial_MP3 Serial
EspSoftwareSerial::UART Serial_MP3;  // this serial port will talk to a Sparkfun MP3 Trigger
EspSoftwareSerial::UART Serial_Dome; // this one will talk to the microcontroller/Marcduino in the dome
#define MP3_TX 22
#define DOME_TX 34
#define CRSF_RX 17
#define CRSF_TX 16
#define CRSF_BAUDRATE 420000 // by default crossfire runs at 420000 baud, if you want to slow it down the receiver and transmitter may need a firmware reflash
// Set up a new Serial object
HardwareSerial crsfSerial(2);
AlfredoCRSF crsf;

//FOOT DRIVE SIGNALS
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
#define leftFootPin 32  
#define rightFootPin 33
Servo leftFootSignal;
Servo rightFootSignal;
#define turnspeed 50    //turnspeed is used by the BHD mixing algorythm. might look into making this more dynamic in the future.
int leftFoot=90;  //will hold foot speed values (0-180)
int rightFoot=90; //will hold foot speed values (0-180)
uint8_t driveSpeeds[3] = {30,80,127};  //for individual R/C, these are the drivespeeds that we can toggle between (0=statue, 127=ludicrous)
uint8_t maxDriveSpeed=0;
#define joystickFootDeadZoneRange 10

//ROAM-A-DOME
#define RAD_RX 26
#define RAD_TX 27
HardwareSerial radSerial(1);
#define numRADChars 9 // data coming from Roam-A-Dome shouldn't be longer than 9 chars (eg. #DP@360\n )
unsigned int RADposition;
uint8_t RADmode;
//unsigned long prevSyRCommandTime;

//DOME SyRen
#define SYREN_ADDR         129  // Serial Address for Dome Syren . 129 dipswitches with lithium cutoff = 000011 or 001011 without lithium cutoff feature
Sabertooth *SyR=new Sabertooth(SYREN_ADDR, radSerial);
unsigned long prevManualDomeMove,prevManualFootMove;
int domeRotationSpeed;
#define invertDome 1
//#define SYREN_PWM_PIN 12 
//Servo domeSignal;

//HEARTBEAT
#define STATUS_PIN 5
unsigned long prevFlipFlop;

#define VBAT_PIN 14
#define VBATR1 68000.0 //voltage divider R1
#define VBATR2 10000.0 //voltage divider R2
#define ADC_RES 4096.0 //8192.0
#define ADC_VLT 3.3
#define VBAT_SCALE 1.098 //1.0932 //if our voltage divider is giving incorrect readings, this can be adjusted
float cap = 0; //not needed, only used for making up telem data to send
#define VBAT_INTERVAL 1000 //after how many milliseconds will we check battery voltage

#define DEBUG 1 //0=no debug info , 1=some debug info (once everything is running well change this to 0)
#define DEBUG_INTERVAL 5000 //after how many milliseconds will we print any debug info to Serial

unsigned long now,prevDebug,prevVBAT;
bool isArmed,prevArmed;
uint8_t speedRange,prevSpeedRange; //0=disarmed 1=slow , 2=medium , 3=FAST!!
int prevTxChannels[17];
int txChannels[17];

uint8_t MP3num,prevMP3num,prevVol;
uint8_t vol = 50; // initial volume level, 0 = full volume, 255 off
#define minVolume 100   //255=silent, anything over 100 is too quiet for my liking so I never go there
#define maxVolume 0     //0=loudest, most obnoxious droid in the room

void setup() {

  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, HIGH);

  Serial.begin(115200);
  Serial.println("COM Serial initialized");
 
  radSerial.begin(9600, SERIAL_8N1, RAD_RX, RAD_TX); //Serial1 is used to talk to Roam-A-Dome or Syren/Sabertooth

  Serial_MP3.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, 0, MP3_TX, false);

  Serial_Dome.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, 0, DOME_TX, false);
  
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX, CRSF_TX);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");

  crsf.begin(crsfSerial);

  //ESP32PWM::allocateTimer(0);
	//ESP32PWM::allocateTimer(1);
	//ESP32PWM::allocateTimer(2);
	//ESP32PWM::allocateTimer(3);
  leftFootSignal.setPeriodHertz(250);
  rightFootSignal.setPeriodHertz(250);
  //domeSignal.setPeriodHertz(250);
  //domeSignal.attach(SYREN_PWM_PIN,1000,2000);

  Serial_MP3.write('t');
  Serial_MP3.write(31); // start off with a laugh

}

void stopFeet() {
    leftFootSignal.write(90);
    rightFootSignal.write(90);
}

void footMotorDrive() {
    if (isArmed==true) {
      mixBHD(map(txChannels[1],1000,2000,0,255), map(txChannels[2],2000,1000,0,255), map(maxDriveSpeed,0,127,90,180)); //put values into leftFoot and rightFoot
      //now supposedly we have good values in leftFoot and rightFoot
      leftFootSignal.write(leftFoot);
      rightFootSignal.write(rightFoot);
    }
}      

//Here is the mixing function, very very heavily based on BigHappyDude's code. This is better explained in comments in previous versions...
void mixBHD(uint8_t stickX, uint8_t stickY, uint8_t maxDriveSpeed){  //maxDriveSpeed should be between 90 and 180
    // This is BigHappyDude's mixing function, for differential (tank) style drive using two motor controllers.
    // Takes a joysticks X and Y values, mixes using the diamond mix, and output a value 0-180 for left and right motors.
    // 180,180 = both feet full speed forward.
    // 000,000 = both feet full speed reverse.
    // 180,000 = left foot full forward, right foot full reverse (spin droid clockwise)
    // 000,180 = left foot full reverse, right foot full forward (spin droid counter-clockwise)
    // 090,090 = no movement
    // for simplicity, we think of this diamond matrix as a range from -100 to +100 , then map the final values to servo range (0-180) at the end
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
       XDist = (map(stickX, 0, 113, -turnspeed, -1));   //  Map the left direction stick value to Turn speed
      } else if(stickX >= 141){
       XDist = (map(stickX, 141, 255, 1, turnspeed));   //  Map the right direction stick value to Turn speed
      }
      //  Constrain to Diamond values.  using 2 line equations and find the intersect, boiled down to the minimum
      //  This was the inspiration; https://github.com/declanshanaghy/JabberBot/raw/master/Docs/Using%20Diamond%20Coordinates%20to%20Power%20a%20Differential%20Drive.pdf
      float TempYDist = YDist;
      float TempXDist = XDist;
      if (YDist>(XDist+100)) {  
        TempXDist = -100/(1-(TempYDist/TempXDist));
        TempYDist = TempXDist+100;
      } else if (YDist>(100-XDist)) {  
        TempXDist = -100/(-1-(TempYDist/TempXDist));
        TempYDist = -TempXDist+100;
      } else if (YDist<(-XDist-100)) {  
        TempXDist = 100/(-1-(TempYDist/TempXDist));
        TempYDist = -TempXDist-100;
      } else if (YDist<(XDist-100)) {
        TempXDist = 100/(1-(TempYDist/TempXDist));
        TempYDist = TempXDist-100;
      }
      float LeftSpeed = ((TempXDist+TempYDist-100)/2)+100;
      LeftSpeed = (LeftSpeed-50)*2;
      float RightSpeed = ((TempYDist-TempXDist-100)/2)+100;
      RightSpeed = (RightSpeed-50)*2;
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

//Use crsf.getChannel(x) to get us channel values (1-16).
void printChannels() {
  for (uint8_t ChannelNum = 1; ChannelNum <= 16; ChannelNum++) {
    Serial.print(ChannelNum);
    Serial.print(":");
    Serial.print(txChannels[ChannelNum]);
    Serial.print(", ");
  }
  Serial.print("L"); Serial.print(leftFoot);
  Serial.print(" R"); Serial.print(rightFoot);
  Serial.println(" ");
}

//send a play command to the MP3 Trigger,
// depending on position of Chan12 (S2) and whether Chan9 (SA/Music) or Chan10 (SD/Beep) is high
// this section should sync up with the lua script on the transmitter
void playMP3() {
  Serial_MP3.write('t');
  // we split Chan12 up into 8 possible values ranging from 1000 to 2000
  // secondary (music) is split into 20 more...
  /*
  201 mus-7Rebels.mp3
  202 mus-8JabbaFlow.mp3
  203 mus-9BinarySunsCoyoteKisses.mp3
  204 mus-10Ookay128.mp3
  205 mus-11CantinaNoizeTank.mp3
  206 mus-12MarchMemox.mp3
  207 mus-13Theme(192kbpsEmpireST).mp3
  208 mus-March192kbps.mp3
  209 mus-16chorusEdit.mp3
  210 mus-trekEditLouder.mp3
  211 mus-Futurama.mp3
  212 mus-Bender.mp3
  213 mus-TMNT.mp3
  214 mus-XMEN.mp3
  215 mus-Farscape.mp3
  216 mus-MrRoboto.mp3
  217 mus-3POrap.mp3
  218 mus-Sabine.mp3
  219 mus-Terminator.mp3
  220 mus-TNG.mp3
  */
  //
  if (txChannels[9]>1800) {
    // channel 12 will be between 1000-2000 and we'll play tune 201-220
    MP3num=map(txChannels[12], 1000, 2000, 201, 211);
  }
  else if (txChannels[14]>1800) {
    MP3num=map(txChannels[12], 1000, 2000, 211, 221);
  }  
  else if (txChannels[12]<=1100) {
      MP3num=random(126,129);        // SCREAM / SHORT CIRCUIT
      while (MP3num == prevMP3num) MP3num=random(126,129); //ensure we don't play the exact same random sound as last time
      Serial_Dome.println();
      Serial_Dome.print("\r@0T4\r"); //send an 0T4 to the dome so R5 can blow his motivator, or R2 can change up his logics  
  }
  else if (txChannels[12]>1100 && txChannels[12]<=1200) {
      MP3num=random(76,80);          // RANDOM SAD BOOP
      while (MP3num == prevMP3num) MP3num=random(76,80); //ensure we don't play the exact same random sound as last time
  }  
  else if (txChannels[12]>1200 && txChannels[12]<=1300) {
      MP3num=180;                    // BEEP CANTINA
  }  
  else if (txChannels[12]>1300 && txChannels[12]<=1400) {
      MP3num=183;                    // WOLF WHISTLE    
  }  
  else if (txChannels[12]>1400 && txChannels[12]<=1500) {
      MP3num=31;                     // LAUGH
  }  
  else if (txChannels[12]>1500 && txChannels[12]<=1600) {
      MP3num=random(185,191);        // RANDOM OOH
      while (MP3num == prevMP3num) MP3num=random(185,191); //ensure we don't play the exact same random sound as last time
  }  
  else if (txChannels[12]>1600 && txChannels[12]<=1700) {
      MP3num=35;                     // DISAGREE
  }  
  else if (txChannels[12]>1700 && txChannels[12]<=1800) {
      MP3num=176;                    // CANTINA
  }  
  else if (txChannels[12]>1800 && txChannels[12]<=1900) {
      MP3num=152;                    // LEIA
  }  
  else {
      MP3num=random(1,20);           // RANDOM BEEP BOOP
      while (MP3num == prevMP3num) MP3num=random(1,20); //ensure we don't play the exact same random sound as last time
  }  
  Serial_MP3.write(MP3num);
  #if DEBUG>0
    //Serial.print("mp3"); 
    Serial.print(txChannels[12]); 
    Serial.print(", mp3"); 
    Serial.println(MP3num); 
  #endif
  prevMP3num=MP3num;
}

void setVolume(uint8_t level) {
  Serial_MP3.write('v');
  Serial_MP3.write(level);
  #if DEBUG>0
    Serial.print("vol");
    Serial.println(level);
  #endif
}

static void sendRxBatteryTelem(float voltage, float current, float capacity, float remaining) {
  crsf_sensor_battery_t crsfBatt = { 0 };
  // Values are MSB first (BigEndian)
  crsfBatt.voltage = htobe16((uint16_t)(voltage * 10.0));   //Volts
  crsfBatt.current = htobe16((uint16_t)(current * 10.0));   //Amps
  crsfBatt.capacity = htobe16((uint16_t)(capacity)) << 8;   //mAh (with this implemetation max capacity is 65535mAh)
  crsfBatt.remaining = (uint8_t)(remaining);                //percent
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
}

void sendGpsTelem(float latitude, float longitude, float groundspeed, float heading, float altitude, float satellites) {
  crsf_sensor_gps_t crsfGps = { 0 };
  // Values are MSB first (BigEndian)
  crsfGps.latitude = htobe32((int32_t)(latitude*10000000.0));
  crsfGps.longitude = htobe32((int32_t)(longitude*10000000.0));
  crsfGps.groundspeed = htobe16((uint16_t)(groundspeed*10.0));
  crsfGps.heading = htobe16((int16_t)(heading*1000.0)); //TODO: heading seems to not display in EdgeTX correctly, some kind of overflow error
  crsfGps.altitude = htobe16((uint16_t)(altitude + 1000.0));
  crsfGps.satellites = (uint8_t)(satellites);
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_GPS, &crsfGps, sizeof(crsfGps));
}

void sendBaroAltitudeTelem(float altitude, float verticalspd) {
  crsf_sensor_baro_altitude_t crsfBaroAltitude = { 0 };
  // Values are MSB first (BigEndian)
  crsfBaroAltitude.altitude = htobe16((uint16_t)(altitude*10.0 + 10000.0));
  //crsfBaroAltitude.verticalspd = htobe16((int16_t)(verticalspd*100.0)); //TODO: fix verticalspd in BaroAlt packets
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BARO_ALTITUDE, &crsfBaroAltitude, sizeof(crsfBaroAltitude) - 2);
  //Supposedly vertical speed can be sent in a BaroAltitude packet, but I cant get this to work.
  //For now I have to send a second vario packet to get vertical speed telemetry to my TX.
  crsf_sensor_vario_t crsfVario = { 0 };
  // Values are MSB first (BigEndian)
  crsfVario.verticalspd = htobe16((int16_t)(verticalspd*100.0));
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_VARIO, &crsfVario, sizeof(crsfVario));
}

void sendAttitudeTelem(float pitch, float roll, float yaw) {
  crsf_sensor_attitude_t crsfAttitude = { 0 };
  // Values are MSB first (BigEndian)
  crsfAttitude.pitch = htobe16((uint16_t)(pitch*10000.0));
  crsfAttitude.roll = htobe16((uint16_t)(roll*10000.0));
  crsfAttitude.yaw = htobe16((uint16_t)(yaw*10000.0));
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_ATTITUDE, &crsfAttitude, sizeof(crsfAttitude));
}

void parseRoamADome() {
    /*get info from Roam-A-Dome
    #DP%c%d\n
    Where %c is:
    ‘@‘: Off
    ‘!’: Home Mode
    ‘$’: Random Mode
    ‘%’: Target Mode
    And %d is the relative home position (0 degrees is front facing).
    \n is newline */
    while (radSerial.available() > 0) {
        static char message[numRADChars];
        static unsigned int message_pos = 0;
        char inByte = radSerial.read();
        //Message coming in (check not terminating character) and guard for over message size
        if ( inByte != '\n' && (message_pos < numRADChars - 1) ) {
          //Add the incoming byte to our message
          message[message_pos] = inByte;
          message_pos++;
        }
        else {
          //Add null character to string
          message[message_pos] = '\0';          
          if (message[0]=='#' && message[1]=='D' && message[2]=='P') {
            //we've got Roam-A-Dome info! parse it to get mode and position
            #if DEBUG>0
              Serial.print("RAD"); 
              Serial.print(message[3]);
              if (message[3]=='!') Serial.print("homing"); 
              else if (message[3]=='$') Serial.print("random");
              else if (message[3]=='%') Serial.print("target");
              else if (message[3]=='@') Serial.print("off");
              RADposition=atoi(&message[4]);
              Serial.println(RADposition);
            #endif
          }
          
          //Reset for the next message
          message_pos = 0;
        }
    }
}

void loop() {
    now=millis();
    // Must call crsf.update() in loop() to process data
    crsf.update();
    if (crsf.isLinkUp()==false) {
      if (isArmed==true) {
        //oh balls, we've just lost connected to the transmitter, stop movement and assume that foot and motor sticks are centered
        #if DEBUG>0
        Serial.println("lost comms while armed!");  
        #endif
        stopFeet();
        SyR->motor(0);
        txChannels[1]=1500; //foot direction
        txChannels[2]=1500; //foot throttle
        txChannels[4]=1500; //dome rotation stick
      }
      isArmed=false; //change armed state asap
      speedRange=0;  //once we arm after re-connecting, we want to make sure we start in slow mode
    }  
    else {
      //we're connected, update all the channels...
      for (int i=1; i<17 ; i++) {
        prevTxChannels[i]=txChannels[i];
        txChannels[i]=crsf.getChannel(i);
        //check the foot drive signals for deadband shenanigans
        for (int i=1; i<3 ; i++) {
          if (txChannels[i]<1500+joystickFootDeadZoneRange && txChannels[i]>1500-joystickFootDeadZoneRange) txChannels[i]=1500;
        }
      }  
      //check arming and speedRange channels...
      //arming=channel5 , speedRange=channel7
      if (txChannels[5]>1500) {
        isArmed=true; 
        if (prevArmed==false) speedRange=1; //always start in slow mode after arming
        if (txChannels[7]<1200 && prevTxChannels[7]>1200) speedRange=1;
        if (txChannels[7]>1200 && txChannels[7]<1700 && (prevTxChannels[7]<1200||prevTxChannels[7]>1700)) speedRange=2;
        if (txChannels[7]>1700 && prevTxChannels[7]<1700) speedRange=3;
        maxDriveSpeed=driveSpeeds[speedRange-1];
      }  
      else {
        isArmed=false; 
        speedRange=0;
      }  
      #if DEBUG>0
        if (prevArmed!=isArmed) {  
          if (isArmed==false) Serial.println ("disarmed"); 
          else Serial.println ("ARMED"); 
        }  
        if (prevSpeedRange!=speedRange) {
          Serial.print("SPD");  
          Serial.println(speedRange);  
        }
      #endif  

      if (isArmed==true && prevArmed==false) {
        leftFootSignal.attach(leftFootPin,1000,2000);
        rightFootSignal.attach(rightFootPin,1000,2000);
      }
      if (isArmed==false && prevArmed==true) {
        stopFeet();
        leftFootSignal.detach();
        rightFootSignal.detach();
      }

      if (txChannels[1]!=prevTxChannels[1] || txChannels[2]!=prevTxChannels[2]) {
        footMotorDrive();
      }  

      //if (prevTxChannels[4]!=txChannels[4]) { //dome stick has changed, send a new value
      if (txChannels[4]>1010 || txChannels[4]<990) { //dome stick is out of deadzone, send a value
        prevManualDomeMove=now;
        #if (invertDome==0)
          domeRotationSpeed=map(txChannels[4],1000,2000,-127,127);
        #else
          domeRotationSpeed=map(txChannels[4],1000,2000,127,-127);
        #endif
        //if (now-prevSyRCommandTime>=50) { //don't yell commands so much
          SyR->motor(domeRotationSpeed);
          //domeSignal.write(map(txChannels[4],1000,2000,0,180));
          //prevSyRCommandTime=now;
        //}  
      }
      if (prevTxChannels[3]!=txChannels[3]) {
        //autodome stick has changed, send a new value
        //when left stick is fully down, value is around 1000 and we won't do autodome at all
        //when left stick is fully up, value is around 2000 and we'll autodome more frequently
        // roam-a-dome
        if (txChannels[3]<=1050 && prevTxChannels[3]>1050) {
          //send command #DPAUTO0 to the RAD (disable dome home mode)
          radSerial.println("#DPAUTO0");
          //radSerial.print('\n');
          #if DEBUG>0
          Serial.println("#DPAUTO0");  
          #endif
        }
        else {
          if (txChannels[3]>=1050 && prevTxChannels[3]<1050) {
            //send command #DPAUTO1 to the RAD (enable dome random auto mode)
            radSerial.println("#DPAUTO1");
            //radSerial.print('\n');
            #if DEBUG>0
            Serial.println("#DPAUTO1");  
            #endif
          }
        }
      }
    }

    #if DEBUG>0
    if (now-prevDebug>=DEBUG_INTERVAL) {
      if (crsf.isLinkUp()) printChannels();
      else Serial.println("no link");  
    }
    #endif

    //see if sound triggering channels have changed...
    if ( (txChannels[9]>1800 && prevTxChannels[9]<1800) || (txChannels[10]>1800 && prevTxChannels[10]<1800) || (txChannels[14]>1800 && prevTxChannels[14]<1800) ) {
      playMP3();
    }
    //check volume...
    if (txChannels[13]!=prevTxChannels[13]) {
      //Serial.println(txChannels[13]); 
      if (txChannels[13]>1980) vol=255; //mute entirely if we're really low
      else if (txChannels[13]<=1000) vol=maxVolume; //this one goes to eleven
      else vol=map(txChannels[13],1000,2000,maxVolume,minVolume);
      if (vol!=prevVol) {
        setVolume(vol);
      }
    }


    if (now-prevVBAT>=VBAT_INTERVAL) {
      int snsVin = analogRead(VBAT_PIN);
      float batteryVoltage = ((float)snsVin * ADC_VLT / ADC_RES) * ((VBATR1 + VBATR2) / VBATR2) * VBAT_SCALE;
      #if DEBUG>0
        if (now-prevDebug>=DEBUG_INTERVAL) {
          Serial.println(batteryVoltage);
        }
      #endif
      sendRxBatteryTelem(batteryVoltage, 1.2, cap += 10, 50); // ***** other than batteryVoltage, these values are made up
    }

    parseRoamADome();

    //vol=map(txChannels[13],1000,2000,maxVolume,minVolume);
    uint8_t volPercent=map(txChannels[13],990,2010,100,0);

    sendGpsTelem(42.12345, -82.12345, volPercent, 20.13, RADposition, speedRange); //mostly made up example nums
    //sendGpsTelem(float latitude, float longitude, float groundspeed, float heading, float altitude, float satellites)
    sendBaroAltitudeTelem(234.1, 154.1); //made up example nums
    sendAttitudeTelem(0.05,-2.43,1.23);  //made up example nums

    //HEARTBEAT LED
    if (now-prevFlipFlop>=1000) {
      prevFlipFlop=now;
      if (digitalRead(STATUS_PIN)==HIGH) digitalWrite(STATUS_PIN, LOW);
      else digitalWrite(STATUS_PIN, HIGH);
    }

    if (now-prevDebug>=DEBUG_INTERVAL) prevDebug=now;
    prevSpeedRange=speedRange;
    prevArmed=isArmed;
    prevVol=vol;
}

