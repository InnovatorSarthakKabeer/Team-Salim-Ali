/*
 * Hexapod Spider Robot — calibration + stand-up + tripod-gait WALKING  (ARDUINO IDE)
 * ESP32 DevKit V1 (Arduino core 3.x) + 2x PCA9685 + 18x MG995
 *
 * Boards : 0x40 (RIGHT legs 0-2)   0x60 (LEFT legs 3-5, A5 bridged)
 * Power  : one XL4015 per board @ 6.0V on V+, grounds common, logic = 3.3V
 * OE     : both PCA9685 OE pins -> GPIO13, 10k pull-up to 3.3V (active-LOW)
 *
 * ---- ARDUINO IDE SETUP ----
 *  Board: "ESP32 Dev Module" (esp32 by Espressif 3.x).
 *  Library: "Adafruit PWM Servo Driver Library" (+ Adafruit BusIO).
 *  Serial Monitor @ 115200, line ending = "Newline".
 *
 * ====================== KEY IDEA: LOGICAL DIRECTIONS ======================
 * Servos are mounted in random mirror orientations, so the SAME command can
 * move two legs opposite ways. We fix this ONCE per leg with a sign table.
 * In software we always use "logical" angles where, for EVERY leg:
 *     +coxa  = swing foot toward FRONT of robot
 *     +femur = push foot DOWN  (raises the body)
 *     +tibia = extend foot DOWN (plants the foot)
 * physical_angle = center[leg][joint] + dir[leg][joint] * logical
 * Calibrate the dir signs with legtest / fdir / tdir / cdir, then 'dump'.
 *
 * ====================== COMMANDS ======================
 *  e                 enable + all legs to flat 90 (staggered, no inrush)
 *  stand / sit       rise to standing / lower to flat
 *  walk / fwd        walk forward (tripod gait)   |  back  walk backward
 *  left / right      turn in place                |  stop  stop & stand
 *  relax             kill outputs, servos limp
 *  --- calibration ---
 *  legtest l         lift ONE leg's corner (l = 0..5) to check its signs
 *  fdir l s          set femur direction  (s = 1 or -1)
 *  tdir l s          set tibia direction
 *  cdir l s          set coxa  direction
 *  trim l j a        set joint center angle (j: 0=coxa 1=femur 2=tibia)
 *  t l j a           raw test: drive leg l joint j to physical angle a
 *  dump              print calibration table (paste back into code)
 *  --- tuning ---
 *  lift n            femur stand push (body height)
 *  bend n            tibia stand bend
 *  swing n           gait stride (coxa sweep)
 *  raise n           gait foot-lift height
 *  speed n           gait phase time in ms (lower = faster)
 *  ?                 menu
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---------------- I2C / boards ----------------
Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40); // RIGHT legs
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x60); // LEFT  legs (A5 bridged)

const int I2C_SDA = 21;
const int I2C_SCL = 22;
const int OE_PIN  = 13;
bool outputsEnabled = false;
bool standing       = false;

// ---------------- servo pulse calibration ----------------
const int SERVO_FREQ = 50;
const int OSC_FREQ   = 27000000;
const int PULSE_MIN  = 110;   // ~0deg   --- CALIBRATE
const int PULSE_MAX  = 490;   // ~180deg --- CALIBRATE

// ---------------- joint / leg model ----------------
enum { COXA = 0, FEMUR = 1, TIBIA = 2 };
struct ServoMap { uint8_t board; uint8_t ch; };

// Board 0 (0x40) = legs 0,1,2 (RIGHT) ; Board 1 (0x60) = legs 3,4,5 (LEFT)
ServoMap smap[18] = {
  {0,0},{0,1},{0,2},  {0,3},{0,4},{0,5},  {0,6},{0,7},{0,8},   // legs 0,1,2
  {1,0},{1,1},{1,2},  {1,3},{1,4},{1,5},  {1,6},{1,7},{1,8},   // legs 3,4,5
};

// ============ PER-LEG CALIBRATION TABLE (edit after 'dump') ============
int coxaCenter[6]  = { 90,90,90, 90,90,90 };
int femurCenter[6] = { 90,90,90, 90,90,90 };
int tibiaCenter[6] = { 90,90,90, 90,90,90 };

// +1 / -1 per leg. THESE ARE GUESSES — calibrate with legtest then fix here.
int coxaDir[6]  = { +1,+1,+1,  -1,-1,-1 };
int femurDir[6] = { -1,-1,-1,  -1,-1,-1 };
int tibiaDir[6] = { +1,+1,+1,  +1,+1,+1 };

// yaw signs for turning (right legs vs left legs oppose). Flip if it turns wrong way.
int yawDir[6]   = { +1,+1,+1,  -1,-1,-1 };

// ---------------- pose / gait amplitudes (tunable live) ----------------
int FEMUR_STAND = 45;   // 'lift' : body height
int TIBIA_STAND = 50;   // 'bend' : foot plant
int LIFT_AMT    = 35;   // 'raise': foot lift during swing
int SWING_AMT   = 22;   // 'swing': coxa stride half-sweep
int STEP_MS     = 200;  // 'speed': time per gait phase

// ---------------- live state (logical degrees) ----------------
float coxaLog[6]={0}, femurLog[6]={0}, tibiaLog[6]={0};
float coxaTgt[6]={0}, femurTgt[6]={0}, tibiaTgt[6]={0};
float curAngle[18];

int tripodA[3] = {0,2,4};   // R-front, R-rear, L-mid
int tripodB[3] = {1,3,5};   // R-mid,  L-front, L-rear

enum WalkMode { FWD, BACK, LEFT, RIGHT };
bool walking = false;
WalkMode walkMode = FWD;

// ================= low-level =================
Adafruit_PWMServoDriver& boardOf(int g){ return smap[g].board==0 ? pwm0 : pwm1; }
int gidx(int leg,int joint){ return leg*3 + joint; }
int angleToCount(int deg){ deg=constrain(deg,0,180); return map(deg,0,180,PULSE_MIN,PULSE_MAX); }

void writeServo(int g,int deg){ boardOf(g).setPWM(smap[g].ch,0,angleToCount(deg)); curAngle[g]=deg; }
void idleServo(int g){ boardOf(g).setPWM(smap[g].ch,0,0); }
void enableOutputs(bool en){ outputsEnabled=en; digitalWrite(OE_PIN, en?LOW:HIGH); }

int physical(int leg,int joint,float logical){
  int c = (joint==COXA)?coxaCenter[leg] : (joint==FEMUR)?femurCenter[leg] : tibiaCenter[leg];
  int d = (joint==COXA)?coxaDir[leg]    : (joint==FEMUR)?femurDir[leg]    : tibiaDir[leg];
  return constrain((int)lround(c + d*logical), 0, 180);
}
void writePhysicalLeg(int l){
  writeServo(gidx(l,COXA),  physical(l,COXA, coxaLog[l]));
  writeServo(gidx(l,FEMUR), physical(l,FEMUR,femurLog[l]));
  writeServo(gidx(l,TIBIA), physical(l,TIBIA,tibiaLog[l]));
}

float ease(float t){ return t*t*(3.0f-2.0f*t); }

// ramp listed legs from current logical to *Tgt simultaneously
void rampLegs(int legs[], int n, int durMs){
  const int steps=30;
  float c0[6],f0[6],t0[6];
  for(int i=0;i<n;i++){ int l=legs[i]; c0[i]=coxaLog[l]; f0[i]=femurLog[l]; t0[i]=tibiaLog[l]; }
  for(int s=1;s<=steps;s++){
    float e=ease((float)s/steps);
    for(int i=0;i<n;i++){
      int l=legs[i];
      coxaLog[l]  = c0[i] + (coxaTgt[l]-c0[i])*e;
      femurLog[l] = f0[i] + (femurTgt[l]-f0[i])*e;
      tibiaLog[l] = t0[i] + (tibiaTgt[l]-t0[i])*e;
      writePhysicalLeg(l);
    }
    delay(durMs/steps);
  }
}
void rampAll(int durMs){ int all[6]={0,1,2,3,4,5}; rampLegs(all,6,durMs); }

void setFemurGroup(int* g,int n,float v){ for(int i=0;i<n;i++) femurTgt[g[i]]=v; }

// ================= actions =================
void doEnableCenter(){
  Serial.println(F("Enable -> flat 90, ONE LEG AT A TIME ..."));
  enableOutputs(true); delay(50);
  for(int l=0;l<6;l++){
    coxaTgt[l]=femurTgt[l]=tibiaTgt[l]=0;
    coxaLog[l]=femurLog[l]=tibiaLog[l]=0;   // assumed assembled at 90
    writePhysicalLeg(l);
    Serial.printf("  leg %d @ 90\n", l);
    delay(300);
  }
  standing=false;
  Serial.println(F("Flat. Calibrate with legtest, then 'stand'."));
}

void doStand(){
  if(!outputsEnabled){ Serial.println(F("Run 'e' first.")); return; }
  Serial.println(F("Standing — diagonal pairs ..."));
  for(int l=0;l<6;l++){ coxaTgt[l]=0; femurTgt[l]=FEMUR_STAND; tibiaTgt[l]=TIBIA_STAND; }
  int pairs[3][2]={{0,5},{3,2},{1,4}};
  for(int p=0;p<3;p++){ int lg[2]={pairs[p][0],pairs[p][1]}; rampLegs(lg,2,600); delay(200); }
  standing=true;
  Serial.println(F("Standing."));
}

void doSit() {
  if (!outputsEnabled) {
    Serial.println(F("Run 'e' first."));
    return;
  }

  Serial.println(F("Sitting ..."));

  // Target sitting position
  for (int l = 0; l < 6; l++) {
    coxaTgt[l]  = 0;
    femurTgt[l] = 0;
    tibiaTgt[l] = 0;
  }

  // Leg pair order from hexapod_walk(2)
  int pairs[3][2] = {
    {1,4},
    {3,2},
    {0,5}
  };

  for (int p = 0; p < 3; p++) {
    int lg[2] = { pairs[p][0], pairs[p][1] };
    rampLegs(lg, 2, 600);
    delay(150);
  }

  standing = false;
  Serial.println(F("Flat (90)."));
}

void doRelax(){
  for(int g=0;g<18;g++) idleServo(g);
  enableOutputs(false); standing=false; walking=false;
  Serial.println(F("RELAXED — limp."));
}

// lift ONE leg's corner using the logical model (verifies that leg's signs)
void doLegTest(int l){
  if(l<0||l>5){ Serial.println(F("leg 0..5")); return; }
  enableOutputs(true);
  Serial.printf("legtest %d : corner should LIFT. If not, flip fdir/tdir %d.\n", l, l);
  coxaTgt[l]=0; femurTgt[l]=FEMUR_STAND; tibiaTgt[l]=TIBIA_STAND;
  int lg[1]={l}; rampLegs(lg,1,600);
}

void doRawTest(int leg,int joint,int angle){
  if(leg<0||leg>5||joint<0||joint>2){ Serial.println(F("bad leg/joint")); return; }
  enableOutputs(true);
  int g=gidx(leg,joint); writeServo(g,constrain(angle,0,180));
  Serial.printf("RAW leg %d joint %d -> %d deg\n", leg,joint,constrain(angle,0,180));
}

void doDump(){
  Serial.println(F("\n// ---- paste into code ----"));
  Serial.print(F("int coxaDir[6]  = {")); for(int i=0;i<6;i++) Serial.printf("%+d%s",coxaDir[i], i<5?",":"};\n");
  Serial.print(F("int femurDir[6] = {")); for(int i=0;i<6;i++) Serial.printf("%+d%s",femurDir[i],i<5?",":"};\n");
  Serial.print(F("int tibiaDir[6] = {")); for(int i=0;i<6;i++) Serial.printf("%+d%s",tibiaDir[i],i<5?",":"};\n");
  Serial.print(F("int coxaCenter[6]  = {")); for(int i=0;i<6;i++) Serial.printf("%d%s",coxaCenter[i], i<5?",":"};\n");
  Serial.print(F("int femurCenter[6] = {")); for(int i=0;i<6;i++) Serial.printf("%d%s",femurCenter[i],i<5?",":"};\n");
  Serial.print(F("int tibiaCenter[6] = {")); for(int i=0;i<6;i++) Serial.printf("%d%s",tibiaCenter[i],i<5?",":"};\n");
  Serial.printf("// FEMUR_STAND=%d TIBIA_STAND=%d SWING=%d RAISE=%d SPEED=%d\n\n",
                FEMUR_STAND,TIBIA_STAND,SWING_AMT,LIFT_AMT,STEP_MS);
}

// ================= WALKING (tripod gait) =================
float sweepVal(int leg,int role){     // role +1 = reposition/advance, -1 = push
  switch(walkMode){
    case FWD:   return role * (+1) * SWING_AMT;
    case BACK:  return role * (-1) * SWING_AMT;
    case LEFT:  return role * (+1) * yawDir[leg] * SWING_AMT;
    case RIGHT: return role * (-1) * yawDir[leg] * SWING_AMT;
  }
  return 0;
}

void pollWalk(){
  static String wb;
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'||c=='\r'){
      if(wb.length()){ String s=wb; wb=""; s.trim(); s.toLowerCase();
        if(s=="stop") walking=false;
        else if(s=="fwd"||s=="walk") walkMode=FWD;
        else if(s=="back")  walkMode=BACK;
        else if(s=="left")  walkMode=LEFT;
        else if(s=="right") walkMode=RIGHT;
        else if(s.startsWith("speed ")){int v; if(sscanf(s.c_str(),"speed %d",&v)==1) STEP_MS=v;}
        else if(s.startsWith("swing ")){int v; if(sscanf(s.c_str(),"swing %d",&v)==1) SWING_AMT=v;}
        else if(s.startsWith("raise ")){int v; if(sscanf(s.c_str(),"raise %d",&v)==1) LIFT_AMT=v;}
      }
    } else wb+=c;
  }
}
bool stopReq(){ pollWalk(); return !walking; }

void gaitCycle(){
  float FS=FEMUR_STAND, FSW=FEMUR_STAND-LIFT_AMT;
  int *A=tripodA, *B=tripodB;

  // --- Half 1: A swings, B pushes ---
  setFemurGroup(A,3,FSW);                                     rampAll(STEP_MS); if(stopReq())return;
  for(int i=0;i<3;i++){ coxaTgt[A[i]]=sweepVal(A[i],+1); coxaTgt[B[i]]=sweepVal(B[i],-1); }
                                                              rampAll(STEP_MS); if(stopReq())return;
  setFemurGroup(A,3,FS);                                      rampAll(STEP_MS); if(stopReq())return;

  // --- Half 2: B swings, A pushes ---
  setFemurGroup(B,3,FSW);                                     rampAll(STEP_MS); if(stopReq())return;
  for(int i=0;i<3;i++){ coxaTgt[B[i]]=sweepVal(B[i],+1); coxaTgt[A[i]]=sweepVal(A[i],-1); }
                                                              rampAll(STEP_MS); if(stopReq())return;
  setFemurGroup(B,3,FS);                                      rampAll(STEP_MS); if(stopReq())return;
}

void startWalk(WalkMode m){
  if(!standing){ Serial.println(F("Type 'stand' first.")); return; }
  walkMode=m; walking=true;
  Serial.println(F("Walking. Type stop / back / left / right / speed n / swing n."));
}

// ================= menu / serial =================
void printMenu(){
  Serial.println(F("\n=========== HEXAPOD ==========="));
  Serial.println(F(" e | stand | sit | relax"));
  Serial.println(F(" walk fwd back left right stop"));
  Serial.println(F(" CAL: legtest l | fdir l s | tdir l s | cdir l s | trim l j a | t l j a | dump"));
  Serial.println(F(" TUNE: lift n | bend n | swing n | raise n | speed n"));
  Serial.println(F(" legs 0=Rf 1=Rm 2=Rr 3=Lf 4=Lm 5=Lr"));
  Serial.println(F("===============================\n"));
}

void handle(String s){
  s.trim(); s.toLowerCase();
  int l,j,a;
  if(s=="e"||s=="enable"||s=="c") doEnableCenter();
  else if(s=="stand") doStand();
  else if(s=="sit")   doSit();
  else if(s=="relax"||s=="off") doRelax();
  else if(s=="walk"||s=="fwd") startWalk(FWD);
  else if(s=="back")  startWalk(BACK);
  else if(s=="left")  startWalk(LEFT);
  else if(s=="right") startWalk(RIGHT);
  else if(s=="stop")  { walking=false; }
  else if(s=="dump")  doDump();
  else if(s=="?"||s=="help") printMenu();
  else if(s.startsWith("legtest ")){ if(sscanf(s.c_str(),"legtest %d",&l)==1) doLegTest(l); }
  else if(s.startsWith("t ")){ if(sscanf(s.c_str(),"t %d %d %d",&l,&j,&a)==3) doRawTest(l,j,a); else Serial.println(F("t l j a")); }
  else if(s.startsWith("fdir ")){ if(sscanf(s.c_str(),"fdir %d %d",&l,&a)==2 && l>=0&&l<6){ femurDir[l]=(a<0)?-1:1; Serial.printf("femurDir[%d]=%+d\n",l,femurDir[l]); } }
  else if(s.startsWith("tdir ")){ if(sscanf(s.c_str(),"tdir %d %d",&l,&a)==2 && l>=0&&l<6){ tibiaDir[l]=(a<0)?-1:1; Serial.printf("tibiaDir[%d]=%+d\n",l,tibiaDir[l]); } }
  else if(s.startsWith("cdir ")){ if(sscanf(s.c_str(),"cdir %d %d",&l,&a)==2 && l>=0&&l<6){ coxaDir[l]=(a<0)?-1:1; Serial.printf("coxaDir[%d]=%+d\n",l,coxaDir[l]); } }
  else if(s.startsWith("trim ")){ if(sscanf(s.c_str(),"trim %d %d %d",&l,&j,&a)==3 && l>=0&&l<6){
        if(j==COXA)coxaCenter[l]=a; else if(j==FEMUR)femurCenter[l]=a; else tibiaCenter[l]=a;
        Serial.printf("center leg%d joint%d=%d\n",l,j,a); } }
  else if(s.startsWith("lift ")) { if(sscanf(s.c_str(),"lift %d",&a)==1) FEMUR_STAND=a; }
  else if(s.startsWith("bend ")) { if(sscanf(s.c_str(),"bend %d",&a)==1) TIBIA_STAND=a; }
  else if(s.startsWith("swing ")){ if(sscanf(s.c_str(),"swing %d",&a)==1) SWING_AMT=a; }
  else if(s.startsWith("raise ")){ if(sscanf(s.c_str(),"raise %d",&a)==1) LIFT_AMT=a; }
  else if(s.startsWith("speed ")){ if(sscanf(s.c_str(),"speed %d",&a)==1) STEP_MS=a; }
  else printMenu();
}

// ================= setup / loop =================
void setup(){
  Serial.begin(115200); delay(300);
  pinMode(OE_PIN,OUTPUT); enableOutputs(false);     // safe: limp before anything
  for(int i=0;i<18;i++) curAngle[i]=90;

  Wire.begin(I2C_SDA,I2C_SCL); Wire.setClock(400000);
  pwm0.begin(); pwm0.setOscillatorFrequency(OSC_FREQ); pwm0.setPWMFreq(SERVO_FREQ);
  pwm1.begin(); pwm1.setOscillatorFrequency(OSC_FREQ); pwm1.setPWMFreq(SERVO_FREQ);
  delay(10);
  for(int g=0;g<18;g++) idleServo(g);

  Serial.println(F("\nHexapod ready. LIMP until 'e'."));
  printMenu();
}

String buf;
void loop(){
  if(walking){
    gaitCycle();
    if(!walking){                               // stopped -> settle to clean stand
      for(int l=0;l<6;l++){ coxaTgt[l]=0; femurTgt[l]=FEMUR_STAND; tibiaTgt[l]=TIBIA_STAND; }
      rampAll(STEP_MS);
      Serial.println(F("Stopped, standing."));
    }
  } else {
    while(Serial.available()){
      char c=Serial.read();
      if(c=='\n'||c=='\r'){ if(buf.length()){ handle(buf); buf=""; } }
      else buf+=c;
    }
  }
}
